#include "config.h"

#if 0                 //ScenSim-Port://
#include <sys/time.h>
#endif                //ScenSim-Port://
#include <time.h>
#include <math.h>
#include <stdarg.h>

#include "core/core.h"
#include "core/mem.h"
#include "core/ip_list.h"
#include "core/ip_set.h"

#if defined(_WIN32) || defined(_WIN64)      //ScenSim-Port://
#define localtime(t)       _localtime32(t)  //ScenSim-Port://
#define strcasecmp(s1, s2) _stricmp(s1, s2) //ScenSim-Port://
#endif                                      //ScenSim-Port://

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup nu_logger
 * @{
 */

/** Checks whether the logger outputs the message or not.
 */
#define nu_logger_need_output(self)    ((self)->output_priority >= (self)->priority)

static void make_tag(nu_logger_t* self);

/*
 */
static const char*
prio_str(const int prio)
{
    if (prio == NU_LOGGER_DEBUG)
        return "DEBUG";
    else if (prio == NU_LOGGER_INFO)
        return "INFO";
    else if (prio == NU_LOGGER_WARN)
        return "WARN";
    else if (prio == NU_LOGGER_ERR)
        return "ERR";
    else if (prio == NU_LOGGER_FATAL)
        return "FATAL";
    else
        return "UNKNOWN";
}

/*
 */
static void
make_tag(nu_logger_t* self)
{
    if (self->format & NU_LOGGER_OUTPUT_NOW) {
        nu_strbuf_appendf(&self->buf, "%ld.%02ld ",
                (long)NU_NOW.tv_sec,
                (long)(NU_NOW.tv_usec / 1e4));
    }
    if (self->format & NU_LOGGER_OUTPUT_TIME) {
#if defined(_WIN32) || defined(_WIN64) //ScenSim-Port://
        long t = (long)time(NULL);     //ScenSim-Port://
#else                                  //ScenSim-Port://
        long t = time(NULL);
#endif                                 //ScenSim-Port://
        struct tm* lt = localtime(&t);
        char tmp[80];
        strftime(tmp, sizeof(tmp), "%F %T ", lt);
        nu_strbuf_append_cstr(&self->buf, tmp);
    }
    if (self->name)
        nu_strbuf_append_cstr(&self->buf, self->name);
    nu_strbuf_append_char(&self->buf, ' ');
    if (self->format & NU_LOGGER_OUTPUT_LEVEL) {
        nu_strbuf_append_cstr(&self->buf, prio_str(self->priority));
        nu_strbuf_append_char(&self->buf, ' ');
    }
    for (int i = 0; i < self->sp; ++i)
        nu_strbuf_append_cstr(&self->buf, self->prefix[i]);
}

/** Initializes logger
 *
 * @param self
 */
PUBLIC void
nu_logger_init(nu_logger_t* self)
{
    self->fp = stderr;
    self->output_priority = NU_LOGGER_WARN;
    self->format = 0;
    self->sp = 0;
    self->name = NULL;
    nu_strbuf_init(&self->buf);
}

/** Destroys the logger
 *
 * @param self
 */
PUBLIC void
nu_logger_destroy(nu_logger_t* self)
{
    nu_strbuf_destroy(&self->buf);
    if (self->name)
        nu_mem_free(self->name);
#ifndef nuOLSRv2_SIMULATOR
    if (self->fp != stderr && self->fp != stdout)
        fclose(self->fp);
#endif
}

/** Outputs log with prefix symbol.
 *
 * @param self
 * @param format
 */
PUBLIC void
nu_logger_log(nu_logger_t* self, const char* format, ...)
{
    if (!nu_logger_need_output(self))
        return;
    va_list ap;
    va_start(ap, format);
    nu_strbuf_clear(&self->buf);
    make_tag(self);
    nu_strbuf_vappendf(&self->buf, format, ap);
    nu_strbuf_append_char(&self->buf, '\n');
    fputs(nu_strbuf_cstr(&self->buf), self->fp);
    va_end(ap);
}

/** Outputs log with prefix symbol.
 *
 * @param self
 * @param strbuf
 */
PUBLIC void
nu_logger_log_strbuf(nu_logger_t* self, const nu_strbuf_t* strbuf)
{
    if (!nu_logger_need_output(self))
        return;

    nu_strbuf_clear(&self->buf);
    make_tag(self);
    nu_strbuf_append_cstr(&self->buf, nu_strbuf_cstr(strbuf));
    nu_strbuf_append_char(&self->buf, '\n');
    fputs(nu_strbuf_cstr(&self->buf), self->fp);
}

/** Outputs log with prefix symbol.
 *
 * @param self
 * @param cstr
 */
PUBLIC void
nu_logger_log_cstr(nu_logger_t* self, const char* cstr)
{
    if (!nu_logger_need_output(self))
        return;

    nu_strbuf_clear(&self->buf);
    make_tag(self);
    nu_strbuf_append_cstr(&self->buf, cstr);
    nu_strbuf_append_char(&self->buf, '\n');
    fputs(nu_strbuf_cstr(&self->buf), self->fp);
}

/** Sets following messages' priority to <code>prio</code>.
 *
 * @param self
 * @param prio
 * @return true if following log messages are outputed.
 */
PUBLIC nu_bool_t
nu_logger_set_prio(nu_logger_t* self, const int prio)
{
    self->priority = prio;
    return self->output_priority >= self->priority;
}

/** Sets the output priority to <code>prio</code>.
 *
 * Following log messages which priority is lower than <code>prio</code>
 * are suppressed.
 *
 * @param self
 * @param prio
 */
PUBLIC void
nu_logger_set_output_prio(nu_logger_t* self, const int prio)
{
    self->output_priority = prio;
    if (self->output_priority > NU_LOGGER_DEBUG)
        self->output_priority = NU_LOGGER_DEBUG;
    if (self->output_priority < NU_LOGGER_ERR)
        self->output_priority = NU_LOGGER_ERR;
}

/** Sets the output priority to <code>prio</code>.
 *
 * @param self
 * @param prio
 */
PUBLIC nu_bool_t
nu_logger_set_output_prio_str(nu_logger_t* self, const char* prio)
{
    if (strcasecmp(prio, "fatal") == 0)
        nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_FATAL);
    else if (strcasecmp(prio, "err") == 0)
        nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_ERR);
    else if (strcasecmp(prio, "error") == 0)
        nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_ERR);
    else if (strcasecmp(prio, "warn") == 0)
        nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_WARN);
    else if (strcasecmp(prio, "info") == 0)
        nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_INFO);
    else if (strcasecmp(prio, "debug") == 0)
        nu_logger_set_output_prio(NU_LOGGER, NU_LOGGER_DEBUG);
    else {
        nu_warn("Unknown log output prio:%s", prio);
        return false;
    }
    return true;
}

/** Increments verbose level.
 *
 * @param self
 */
PUBLIC void
nu_logger_set_more_verbose(nu_logger_t* self)
{
    nu_logger_set_output_prio(self, self->output_priority + 1);
}

/** Decrements verbose level.
 *
 * @param self
 */
PUBLIC void
nu_logger_set_more_quiet(nu_logger_t* self)
{
    nu_logger_set_output_prio(self, self->output_priority - 1);
}

/** Sets file pointer.
 *
 * @param self
 * @param fp
 */
PUBLIC void
nu_logger_set_fp(nu_logger_t* self, FILE* fp)
{
    self->fp = fp;
}

/** Sets log output format.
 *
 * @param self
 * @param fmt
 */
PUBLIC void
nu_logger_set_format(nu_logger_t* self, const uint8_t fmt)
{
    self->format = fmt;
}

/** Sets program name for logging.
 *
 * @param self
 * @param name
 */
PUBLIC void
nu_logger_set_name(nu_logger_t* self, const char* name)
{
    if (self->name)
        nu_mem_free(self->name);
    self->name = nu_mem_strdup(name);
}

/**
 * @param self
 * @param prefix
 */
PUBLIC void
nu_logger_push_prefix(nu_logger_t* self, const char* prefix)
{
    assert(self->sp < NU_LOGGER_PREFIX_MAX_NUM);
    self->prefix[self->sp++] = prefix;
}

/**
 * @param self
 */
PUBLIC void
nu_logger_pop_prefix(nu_logger_t* self)
{
    assert(self->sp > 0);
    --self->sp;
}

/**
 * @param self
 */
PUBLIC void
nu_logger_clear_prefix(nu_logger_t* self)
{
    self->sp = 0;
}

/** @} */

}//namespace// //ScenSim-Port://
