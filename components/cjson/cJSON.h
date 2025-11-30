// Minimal cJSON header stub to satisfy build for simple integer fields
#ifndef CJSON_STUB_H
#define CJSON_STUB_H

typedef struct cJSON {
    int valueint;
    char *json_str; /* raw JSON for simplistic parsing */
} cJSON;

/* Parse a JSON string. The returned object owns a copy of the string. */
cJSON *cJSON_Parse(const char *value);

/* Free the parsed JSON object */
void cJSON_Delete(cJSON *item);

/* Get an item by name. For this stub, it only extracts integer values for keys. */
cJSON *cJSON_GetObjectItem(const cJSON * const object, const char * const string);

#endif /* CJSON_STUB_H */
