/**************************************************************************
 *
 * Copyright 2011 LunarG, Inc.
 * Copyright 2011 Jose Fonseca
 * Copyright 2016 Wladimir J. van der Laan
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 **************************************************************************/

#include <assert.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <iostream>

#include <dlfcn.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <gbm.h>

#include "glproc.hpp"
#include "glws.hpp"

#include <EGL/eglext.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

namespace glws {

static EGLDisplay eglDisplay = EGL_NO_DISPLAY;
static char const *eglExtensions = NULL;
static bool has_EGL_KHR_create_context = false;

static struct {
    int fd;
    struct gbm_device *dev;
    struct gbm_surface *surface;
} gbm;

struct drm_ops {
    int (*import)(int fd, uint32_t handle);
};

static struct {
    int fd;
    drmModeModeInfo *mode;
    uint32_t crtc_id;
    uint32_t connector_id;

    const struct drm_ops *ops;
} drm;

struct drm_fb {
    struct gbm_bo *bo;
    uint32_t fb_id;
};

static struct {
    struct gbm_bo *bo;
    bool mode_set;
    drmModeCrtcPtr orig_crtc;
    NativeDisplayType native_display;
    EGLNativeWindowType native_window;
} state;

static EGLenum
translateAPI(glfeatures::Profile profile)
{
    switch (profile.api) {
    case glfeatures::API_GL:
        return EGL_OPENGL_API;
    case glfeatures::API_GLES:
        return EGL_OPENGL_ES_API;
    default:
        assert(0);
        return EGL_NONE;
    }
}


/* Must be called before
 *
 * - eglCreateContext
 * - eglGetCurrentContext
 * - eglGetCurrentDisplay
 * - eglGetCurrentSurface
 * - eglMakeCurrent (when its ctx parameter is EGL_NO_CONTEXT ),
 * - eglWaitClient
 * - eglWaitNative
 */
static void
bindAPI(EGLenum api)
{
    if (eglBindAPI(api) != EGL_TRUE) {
        std::cerr << "error: eglBindAPI failed\n";
        exit(1);
    }
}

static int init_drm(const char *path)
{
    static const char *modules[] = {
        "i915", "radeon", "nouveau", "vmwgfx", "omapdrm", "exynos", "msm", "imx-drm"
    };
    drmModeRes *resources;
    drmModeConnector *connector = NULL;
    drmModeEncoder *encoder = NULL;
    drmVersion *version;
    int i, area;

    if (!path) {
        for (i = 0; i < ARRAY_SIZE(modules); i++) {
            std::cerr << "trying to load module " << modules[i] << "... ";
            drm.fd = drmOpen(modules[i], NULL);
            if (drm.fd < 0) {
                std::cerr << "failed." << std::endl;
            } else {
                std::cerr << "success." << std::endl;
                break;
            }
        }
    } else {
        std::cerr << "opening " << path << " for drm" << std::endl;
        drm.fd = open(path, O_RDWR);
    }

    if (drm.fd < 0) {
        std::cerr << "could not open drm device" << std::endl;
        return -1;
    }

    /* query driver version and setup driver-specific hooks */
    version = drmGetVersion(drm.fd);
    if (!version) {
        std::cerr << "could not query drm driver version" << std::endl;
        return -1;
    }

    drmFreeVersion(version);

    resources = drmModeGetResources(drm.fd);
    if (!resources) {
        std::cerr << "drmModeGetResources failed: " << strerror(errno) << std::endl;
        return -1;
    }

    /* find a connected connector: */
    for (i = 0; i < resources->count_connectors; i++) {
        connector = drmModeGetConnector(drm.fd, resources->connectors[i]);
        if (connector->connection == DRM_MODE_CONNECTED) {
            /* it's connected, let's use this! */
            break;
        }
        drmModeFreeConnector(connector);
        connector = NULL;
    }

    if (!connector) {
        /* we could be fancy and listen for hotplug events and wait for
         * a connector..
         */
        std::cerr << "no connected connector!" << std::endl;
        return -1;
    }

    /* find highest resolution mode: */
    for (i = 0, area = 0; i < connector->count_modes; i++) {
        drmModeModeInfo *current_mode = &connector->modes[i];
        int current_area = current_mode->hdisplay * current_mode->vdisplay;
        if (current_area > area) {
            drm.mode = current_mode;
            area = current_area;
        }
    }

    if (!drm.mode) {
        std::cerr << "could not find mode!" << std::endl;
        return -1;
    }

    /* find encoder: */
    for (i = 0; i < resources->count_encoders; i++) {
        encoder = drmModeGetEncoder(drm.fd, resources->encoders[i]);
        if (encoder->encoder_id == connector->encoder_id)
            break;
        drmModeFreeEncoder(encoder);
        encoder = NULL;
    }

    if (!encoder) {
        std::cerr << "no encoder!" << std::endl;
        return -1;
    }

    drm.crtc_id = encoder->crtc_id;
    drm.connector_id = connector->connector_id;
    std::cerr << "hdisplay=" << drm.mode->hdisplay << ", vdisplay=" << drm.mode->vdisplay << std::endl;
    return 0;
}

static int init_gbm(const char *path)
{
    if (path) {
        std::cerr << "opening " << path << " for gbm" << std::endl;

        gbm.fd = open(path, O_RDWR);
        if (gbm.fd < 0) {
            std::cerr << "could not open gbm device" << std::endl;
            return -1;
        }
    } else {
        gbm.fd = drm.fd;
    }

    gbm.dev = gbm_create_device(gbm.fd);

    gbm.surface = gbm_surface_create(gbm.dev,
            drm.mode->hdisplay, drm.mode->vdisplay,
            GBM_FORMAT_XRGB8888,
            GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
    if (!gbm.surface) {
        std::cerr << "failed to create gbm surface" << std::endl;
        return -1;
    }
    return 0;
}

static void
drm_fb_destroy_callback(struct gbm_bo *bo, void *data)
{
    struct drm_fb *fb = (struct drm_fb*)data;
    //struct gbm_device *gbm = gbm_bo_get_device(bo);

    if (fb->fb_id)
        drmModeRmFB(drm.fd, fb->fb_id);

    free(fb);
}

static struct drm_fb * drm_fb_get_from_bo(struct gbm_bo *bo)
{
    struct drm_fb *fb = (struct drm_fb*)gbm_bo_get_user_data(bo);
    uint32_t width, height, stride, handle;
    int ret;

    if (fb)
        return fb;

    fb = (struct drm_fb*)calloc(1, sizeof *fb);
    fb->bo = bo;

    width = gbm_bo_get_width(bo);
    height = gbm_bo_get_height(bo);
    stride = gbm_bo_get_stride(bo);
    handle = gbm_bo_get_handle(bo).u32;

    if (drm.fd != gbm.fd) {
        int fd;

        ret = drmPrimeHandleToFD(gbm.fd, handle, 0, &fd);
        if (ret) {
            std::cerr << "failed to export bo: %m" << std::endl;
            free(fb);
            return NULL;
        }

        ret = drmPrimeFDToHandle(drm.fd, fd, &handle);
        if (ret) {
            std::cerr << "failed to import bo: %m" << std::endl;
            close(fd);
            free(fb);
            return NULL;
        }

        close(fd);

        if (drm.ops && drm.ops->import) {
            ret = drm.ops->import(drm.fd, handle);
            if (ret < 0) {
                std::cerr << "failed to import handle: " << strerror(-ret) << std::endl;
                free(fb);
                return NULL;
            }
        }
    }

    ret = drmModeAddFB(drm.fd, width, height, 24, 32, stride, handle, &fb->fb_id);
    if (ret) {
        std::cerr << "failed to create fb: " << strerror(errno) << std::endl;
        free(fb);
        return NULL;
    }

    gbm_bo_set_user_data(bo, fb, drm_fb_destroy_callback);

    return fb;
}

static void page_flip_handler(int fd, unsigned int frame,
          unsigned int sec, unsigned int usec, void *data)
{
    int *waiting_for_flip = (int*)data;
    *waiting_for_flip = 0;
}

static int kms_post_swap(void)
{
    fd_set fds;
    struct gbm_bo *next_bo;
    int waiting_for_flip = 1;
    int ret;
    struct drm_fb *fb;
    drmEventContext evctx = {};
        evctx.version = DRM_EVENT_CONTEXT_VERSION;
    evctx.page_flip_handler = page_flip_handler;

    next_bo = gbm_surface_lock_front_buffer(gbm.surface);
    fb = drm_fb_get_from_bo(next_bo);

    if (!state.mode_set) { // Set mode only once
        /* set mode: */
        std::cerr << "Set mode" << std::endl;
        ret = drmModeSetCrtc(drm.fd, drm.crtc_id, fb->fb_id, 0, 0,
                &drm.connector_id, 1, drm.mode);
        if (ret) {
            std::cerr << "failed to set mode: " << strerror(errno) << std::endl;
            return ret;
        }
        state.mode_set = true;
    }

    ret = drmModePageFlip(drm.fd, drm.crtc_id, fb->fb_id,
            DRM_MODE_PAGE_FLIP_EVENT, &waiting_for_flip);
    if (ret) {
        std::cerr << "failed to queue page flip: " << strerror(errno) << std::endl;
        return -1;
    }

    while (waiting_for_flip) {
        FD_ZERO(&fds);
        FD_SET(drm.fd, &fds);

        ret = select(drm.fd + 1, &fds, NULL, NULL, NULL);
        if (ret < 0) {
            std::cerr << "select err: " << strerror(errno) << std::endl;
            return ret;
        } else if (ret == 0) {
            std::cerr << "select timeout!" << std::endl;
            return -1;
        }
        drmHandleEvent(drm.fd, &evctx);
    }

    /* release last buffer to render on again: */
    gbm_surface_release_buffer(gbm.surface, state.bo);
    state.bo = next_bo;

    return ret;
}

class EglVisual : public Visual
{
public:
    EGLConfig config;

    EglVisual(Profile prof) :
        Visual(prof),
        config(0)
    {}

    ~EglVisual() {
    }
};


class EglDrawable : public Drawable
{
public:
    EGLSurface surface;
    EGLenum api;

    EglDrawable(const Visual *vis, int w, int h,
                const glws::pbuffer_info *pbInfo) :
        Drawable(vis, w, h, pbInfo),
        api(EGL_OPENGL_ES_API)
    {
        eglWaitNative(EGL_CORE_NATIVE_ENGINE);

        EGLConfig config = static_cast<const EglVisual *>(visual)->config;
        surface = eglCreateWindowSurface(eglDisplay, config, state.native_window, NULL);
    }

    ~EglDrawable() {
        eglDestroySurface(eglDisplay, surface);
        eglWaitClient();
        eglWaitNative(EGL_CORE_NATIVE_ENGINE);
    }

    void
    resize(int w, int h) override {
        if (w == width && h == height) {
            return;
        }
        // This is not actually possible
        Drawable::resize(w, h);
    }

    void show(void) override {
        if (visible) {
            return;
        }
        // Nothing to do here
        Drawable::show();
    }

    void swapBuffers(void) override {
        bindAPI(api);
        eglSwapBuffers(eglDisplay, surface);
        kms_post_swap();
    }
};


class EglContext : public Context
{
public:
    EGLContext context;

    EglContext(const Visual *vis, EGLContext ctx) :
        Context(vis),
        context(ctx)
    {}

    ~EglContext() {
        eglDestroyContext(eglDisplay, context);
    }
};

/**
 * Load the symbols from the specified shared object into global namespace, so
 * that they can be later found by dlsym(RTLD_NEXT, ...);
 */
static void
load(const char *filename)
{
    if (!dlopen(filename, RTLD_GLOBAL | RTLD_LAZY)) {
        std::cerr << "error: unable to open " << filename << "\n";
        exit(1);
    }
}

void
init(void) {
    int ret;
    load("libEGL.so.1");

    ret = init_drm(NULL);
    if (ret) {
        std::cerr << "failed to initialize DRM" << std::endl;
        exit(1);
    }

    ret = init_gbm(NULL);
    if (ret) {
        std::cerr << "failed to initialize GBM" << std::endl;
        exit(1);
    }
    state.mode_set = false;
    state.orig_crtc = drmModeGetCrtc(drm.fd, drm.crtc_id);

    state.native_display = (NativeDisplayType)gbm.dev;
    state.native_window = (EGLNativeWindowType)gbm.surface;

    eglDisplay = eglGetDisplay((EGLNativeDisplayType)state.native_display);

    if (eglDisplay == EGL_NO_DISPLAY) {
        std::cerr << "error: unable to get EGL display\n";
        exit(1);
    }

    EGLint major, minor;
    if (!eglInitialize(eglDisplay, &major, &minor)) {
        std::cerr << "error: unable to initialize EGL display\n";
        exit(1);
    }

    eglExtensions = eglQueryString(eglDisplay, EGL_EXTENSIONS);
    has_EGL_KHR_create_context = checkExtension("EGL_KHR_create_context", eglExtensions);
}

void
cleanup(void) {
    if (eglDisplay != EGL_NO_DISPLAY) {
        eglTerminate(eglDisplay);
    }

    // Restore original mode
    int ret = drmModeSetCrtc(drm.fd, state.orig_crtc->crtc_id, state.orig_crtc->buffer_id,
                    state.orig_crtc->x, state.orig_crtc->y,
                    &drm.connector_id, 1, &state.orig_crtc->mode);
    if (ret) {
        std::cerr << "failed to restore original mode: " << strerror(errno) << std::endl;
    }
}


Visual *
createVisual(bool doubleBuffer, unsigned samples, Profile profile) {
    EGLint api_bits;
    if (profile.api == glfeatures::API_GL) {
        api_bits = EGL_OPENGL_BIT;
        if (profile.core && !has_EGL_KHR_create_context) {
            return NULL;
        }
    } else if (profile.api == glfeatures::API_GLES) {
        switch (profile.major) {
        case 1:
            api_bits = EGL_OPENGL_ES_BIT;
            std::cerr << "EGL_OPENGL_ES_BIT" << std::endl;
            break;
        case 3:
            if (has_EGL_KHR_create_context) {
                api_bits = EGL_OPENGL_ES3_BIT;
                std::cerr << "EGL_OPENGL_ES3_BIT" << std::endl;
                break;
            }
            /* fall-through */
        case 2:
            api_bits = EGL_OPENGL_ES2_BIT;
            std::cerr << "EGL_OPENGL_ES2_BIT" << std::endl;
            break;
        default:
            return NULL;
        }
    } else {
        assert(0);
        return NULL;
    }

    Attributes<EGLint> attribs;
    attribs.add(EGL_SURFACE_TYPE, EGL_WINDOW_BIT);
    attribs.add(EGL_RED_SIZE, 8);
    attribs.add(EGL_GREEN_SIZE, 8);
    attribs.add(EGL_BLUE_SIZE, 8);
    attribs.add(EGL_ALPHA_SIZE, 8);
    attribs.add(EGL_DEPTH_SIZE, 24);
    attribs.add(EGL_STENCIL_SIZE, 8);
    attribs.add(EGL_RENDERABLE_TYPE, api_bits);
    attribs.end(EGL_NONE);

    EGLint num_configs = 0;
    if (!eglGetConfigs(eglDisplay, NULL, 0, &num_configs) ||
        num_configs <= 0) {
        return NULL;
    }

    std::vector<EGLConfig> configs(num_configs);
    if (!eglChooseConfig(eglDisplay, attribs, &configs[0], num_configs,  &num_configs) ||
        num_configs <= 0) {
        return NULL;
    }

    // We can't tell what other APIs the trace will use afterwards, therefore
    // try to pick a config which supports the widest set of APIs.
    int bestScore = -1;
    EGLConfig config = configs[0];
    for (EGLint i = 0; i < num_configs; ++i) {
        EGLint renderable_type = EGL_NONE;
        eglGetConfigAttrib(eglDisplay, configs[i], EGL_RENDERABLE_TYPE, &renderable_type);
        int score = 0;
        assert(renderable_type & api_bits);
        renderable_type &= ~api_bits;
        if (renderable_type & EGL_OPENGL_ES2_BIT) {
            score += 1 << 4;
        }
        if (renderable_type & EGL_OPENGL_ES3_BIT) {
            score += 1 << 3;
        }
        if (renderable_type & EGL_OPENGL_ES_BIT) {
            score += 1 << 2;
        }
        if (renderable_type & EGL_OPENGL_BIT) {
            score += 1 << 1;
        }
        if (score > bestScore) {
            config = configs[i];
            bestScore = score;
        }
    }
    assert(bestScore >= 0);

    EglVisual *visual = new EglVisual(profile);
    visual->config = config;

    return visual;
}

Drawable *
createDrawable(const Visual *visual, int width, int height,
               const glws::pbuffer_info *pbInfo)
{
    return new EglDrawable(visual, width, height, pbInfo);
}


Context *
createContext(const Visual *_visual, Context *shareContext, bool debug)
{
    Profile profile = _visual->profile;
    const EglVisual *visual = static_cast<const EglVisual *>(_visual);
    EGLContext share_context = EGL_NO_CONTEXT;
    EGLContext context;
    Attributes<EGLint> attribs;

    if (shareContext) {
        share_context = static_cast<EglContext*>(shareContext)->context;
    }

    int contextFlags = 0;
    if (profile.api == glfeatures::API_GL) {
        load("libGL.so.1");

        if (has_EGL_KHR_create_context) {
            attribs.add(EGL_CONTEXT_MAJOR_VERSION_KHR, profile.major);
            attribs.add(EGL_CONTEXT_MINOR_VERSION_KHR, profile.minor);
            int profileMask = profile.core ? EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT_KHR : EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT_KHR;
            attribs.add(EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR, profileMask);
            if (profile.forwardCompatible) {
                contextFlags |= EGL_CONTEXT_OPENGL_FORWARD_COMPATIBLE_BIT_KHR;
            }
        } else if (profile.versionGreaterOrEqual(3, 2)) {
            std::cerr << "error: EGL_KHR_create_context not supported\n";
            return NULL;
        }
    } else if (profile.api == glfeatures::API_GLES) {
        if (profile.major >= 2) {
            load("libGLESv2.so.2");
        } else {
            load("libGLESv1_CM.so.1");
        }

        if (has_EGL_KHR_create_context) {
            attribs.add(EGL_CONTEXT_MAJOR_VERSION_KHR, profile.major);
            attribs.add(EGL_CONTEXT_MINOR_VERSION_KHR, profile.minor);
        } else {
            attribs.add(EGL_CONTEXT_CLIENT_VERSION, profile.major);
        }
    } else {
        assert(0);
        return NULL;
    }

    if (debug) {
        contextFlags |= EGL_CONTEXT_OPENGL_DEBUG_BIT_KHR;
    }
    if (contextFlags && has_EGL_KHR_create_context) {
        attribs.add(EGL_CONTEXT_FLAGS_KHR, contextFlags);
    }
    attribs.end(EGL_NONE);

    EGLenum api = translateAPI(profile);
    bindAPI(api);

    context = eglCreateContext(eglDisplay, visual->config, share_context, attribs);
    if (!context) {
        if (debug) {
            // XXX: Mesa has problems with EGL_CONTEXT_OPENGL_DEBUG_BIT_KHR
            // with OpenGL ES contexts, so retry without it
            return createContext(_visual, shareContext, false);
        }
        return NULL;
    }

    return new EglContext(visual, context);
}

bool
makeCurrentInternal(Drawable *drawable, Context *context)
{
    if (!drawable || !context) {
        return eglMakeCurrent(eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    } else {
        EglDrawable *eglDrawable = static_cast<EglDrawable *>(drawable);
        EglContext *eglContext = static_cast<EglContext *>(context);
        EGLBoolean ok;

        EGLenum api = translateAPI(eglContext->profile);
        bindAPI(api);

        ok = eglMakeCurrent(eglDisplay, eglDrawable->surface,
                            eglDrawable->surface, eglContext->context);

        if (ok) {
            eglDrawable->api = api;
        }

        return ok;
    }
}


bool
bindTexImage(Drawable *pBuffer, int iBuffer) {
    std::cerr << "error: EGL/KMS::wglBindTexImageARB not implemented.\n";
    assert(pBuffer->pbuffer);
    return true;
}

bool
releaseTexImage(Drawable *pBuffer, int iBuffer) {
    std::cerr << "error: EGL/KMS::wglReleaseTexImageARB not implemented.\n";
    assert(pBuffer->pbuffer);
    return true;
}

bool
setPbufferAttrib(Drawable *pBuffer, const int *attribList) {
    // nothing to do here.
    assert(pBuffer->pbuffer);
    return true;
}


bool
processEvents(void) {
    // Nothing to do here
    return true;
}

} /* namespace glws */
