#include "config.h"
#include <errno.h>
#include <ctype.h>

#include "olsrv2/olsrv2.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup @olsrv2-metric
 * @{
 */

/**
 * Link metric
 */
#if defined(_WIN32) || defined(_WIN64)         //ScenSim-Port://
typedef struct _metric {                       //ScenSim-Port://
    struct _metric*   next;  ///< next element //ScenSim-Port://
    struct _metric*   prev;  ///< prev element //ScenSim-Port://
#else                                          //ScenSim-Port://
typedef struct metric {
    struct metric*   next;   ///< next element
    struct metric*   prev;   ///< prev element
#endif                                         //ScenSim-Port://

    nu_ip_t          src;    ///< src ip
    nu_ip_t          dst;    ///< dest ip
    nu_link_metric_t metric; ///< metric value
} metric_t;

/** Destructor */
#define metric_free    nu_mem_free

}//namespace// //ScenSim-Port://

#include "olsrv2/metric_list_.h"

namespace NuOLSRv2Port { //ScenSim-Port://

/**
 * Initializes link metric list.
 */
PUBLIC void
olsrv2_metric_list_init(void)
{
    metric_list_init(&OLSR->metric_list);
}

/**
 * Destroys link metric list.
 */
PUBLIC void
olsrv2_metric_list_destroy(void)
{
    metric_list_remove_all(&OLSR->metric_list);
    metric_list_destroy(&OLSR->metric_list);
}

/**
 * Gets link metric.
 *
 * @param tuple_i
 *	Interface
 * @param src
 *	UDP packet's source address
 */
PUBLIC nu_link_metric_t
olsrv2_metric_list_get(tuple_i_t* tuple_i, const nu_ip_t src)
{
    FOREACH_METRIC_LIST(m, &OLSR->metric_list) {
        if (nu_ip_eq_without_prefix(m->src, src) &&
            nu_ip_set_contain_without_prefix(&tuple_i->local_ip_list,
                    m->dst)) {
            OLSRV2_DO_LOG(link_metric,
                    nu_logger_log(NU_LOGGER, "Static metric:%I -> %I:%f",
                            tuple_i_local_ip(tuple_i), src, m->metric);
                    );
            return m->metric;
        }
    }
    OLSRV2_DO_LOG(link_metric,
            nu_logger_log(NU_LOGGER, "No static metric:%I -> %I",
                    tuple_i_local_ip(tuple_i), src);
            );
    return UNDEF_METRIC;
}

/**
 * Adds link metric.
 *
 * @param src
 * @param dst
 * @param metric
 */
static void
metric_list_add(const nu_ip_t src, const nu_ip_t dst,
        nu_link_metric_t metric)
{
    FOREACH_METRIC_LIST(m, &OLSR->metric_list) {
        if (nu_ip_eq_without_prefix(m->src, src) &&
            nu_ip_eq_without_prefix(m->dst, dst)) {
            m->metric = metric;
            return;
        }
    }
    metric_t* n = nu_mem_alloc(metric_t);
    n->src = src;
    n->dst = dst;
    n->metric = metric;
    metric_list_insert_tail(&OLSR->metric_list, n);
}

static char*
get_ip(char* p, nu_ip_t* ip)
{
    while (isspace(*p))
        ++p;
    if (*p == '\0') {
        nu_fatal("metric file format error.");
        /* NOTREACH */
    }
    char* q = p;
    while (!isspace(*q))
        ++q;

    char save = *q;
    *q  = '\0';
    *ip = nu_ip_create_with_str(p, NU_AF);
    *q  = save;
    return q;
}

static char*
get_dir(char* p, char* dir)
{
    while (isspace(*p))
        ++p;
    if (*p == '\0') {
        nu_fatal("metric file format error.");
        /* NOTREACH */
    }
    *dir = *p;
    while (!isspace(*p))
        ++p;
    return p;
}

static char*
get_metric(char* p, nu_link_metric_t* metric)
{
    while (isspace(*p))
        ++p;
    if (*p == '\0') {
        nu_fatal("metric file format error.");
        /* NOTREACH */
    }
    char* q = p;
    while (!isspace(*q))
        ++q;

    char save = *q;
    *q = '\0';
    *metric = atof(p);
    *q = save;
    return q;
}

/** Load the link metric list.
 *
 * @param file
 */
PUBLIC void
olsrv2_metric_list_load(const char* file)
{
    FILE* fp;
    if ((fp = fopen(file, "r")) == NULL) {
        nu_fatal("%s:%s", strerror(errno), file);
        /* NOTREACH */
    }

    char buf[128];
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        if (buf[0] == '#')
            continue;
        char* p = buf;
        while (isspace(*p))
            ++p;
        if (*p == '0')
            continue;
        nu_ip_t src;
        p = get_ip(p, &src);
        nu_ip_t dst;
        p = get_ip(p, &dst);
        char dir;
        p = get_dir(p, &dir);
        nu_link_metric_t metric;
        get_metric(p, &metric);
        if (metric < MINIMUM_METRIC || metric > MAXIMUM_METRIC) {
            nu_fatal("metric range error (metric must be [%f:%f])",
                    MINIMUM_METRIC, MAXIMUM_METRIC);
            /* NOTREACH */
        }
        metric_list_add(src, dst, metric);
        if (dir == 'B' || dir == 'b')
            metric_list_add(dst, src, metric);
    }
    fclose(fp);
}

/** Outputs metric list.
 *
 * @param logger
 */
PUBLIC void
olsrv2_metric_list_put_log(nu_logger_t* logger)
{
    nu_logger_log(logger, "METRIC_LIST:%d:--", metric_list_size(&OLSR->metric_list));
    FOREACH_METRIC_LIST(m, &OLSR->metric_list) {
        nu_logger_log(logger, "METRIC_LIST:%I:%I:%g",
                m->src, m->dst, m->metric);
    }
}

/** @} */

}//namespace// //ScenSim-Port://
