/*
 * Minimal cJSON stub implementation sufficient for the simple usage in main.c
 * This is NOT a full JSON parser. It only supports parsing integer values for
 * top-level keys like "tank_id" and "target_amount" used by this project.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

static char *strdup_safe(const char *s)
{
    if (!s) return NULL;
    size_t n = strlen(s) + 1;
    char *p = malloc(n);
    if (p) memcpy(p, s, n);
    return p;
}

cJSON *cJSON_Parse(const char *value)
{
    if (!value) return NULL;
    cJSON *obj = (cJSON *)malloc(sizeof(cJSON));
    if (!obj) return NULL;
    obj->valueint = 0;
    obj->json_str = strdup_safe(value);
    return obj;
}

void cJSON_Delete(cJSON *item)
{
    if (!item) return;
    if (item->json_str) free(item->json_str);
    free(item);
}

/* Very small helper to locate a key and parse an integer following ':' */
static int parse_int_for_key(const char *json, const char *key, int *out)
{
    if (!json || !key || !out) return 0;
    const char *p = strstr(json, key);
    if (!p) return 0;
    /* find ':' after the key */
    p = strchr(p, ':');
    if (!p) return 0;
    p++; /* move past ':' */
    /* skip spaces */
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == '"' || *p == '\'') p++;
    int val = 0;
    if (sscanf(p, "%d", &val) == 1) {
        *out = val;
        return 1;
    }
    return 0;
}

cJSON *cJSON_GetObjectItem(const cJSON * const object, const char * const string)
{
    if (!object || !object->json_str || !string) return NULL;
    cJSON *res = (cJSON *)malloc(sizeof(cJSON));
    if (!res) return NULL;
    res->json_str = NULL;
    res->valueint = 0;
    if (!parse_int_for_key(object->json_str, string, &res->valueint)) {
        /* no integer found; free and return NULL to mimic real behaviour */
        free(res);
        return NULL;
    }
    return res;
}
