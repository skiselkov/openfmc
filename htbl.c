#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "libxxhash.h"
#include "helpers.h"
#include "htbl.h"

#if	defined(__x86_64__)
#define	XXH(x, y)	XXH64((x), (y), 0)
#elif	defined(__i386__)
#define	XXH(x, y)	XXH32((x), (y), 0)
#else
#error	"Unsupported platform. Please add a 32/64-bit determination switch."
#endif

void
htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz)
{
	assert(key_sz != 0);
	assert(tbl_sz != 0);
	assert(tbl_sz <= ((size_t)1 << ((sizeof(size_t) * 8) - 1)));

	memset(htbl, 0, sizeof (*htbl));
	/* round table size up to nearest multiple of 2 */
	tbl_sz = P2ROUNDUP(tbl_sz);
	htbl->mask = tbl_sz - 1;
	htbl->buckets = malloc(sizeof (*htbl->buckets) * tbl_sz);
	assert(htbl->buckets != NULL);
	for (size_t i = 0; i < tbl_sz; i++)
		list_create(&htbl->buckets[i], sizeof (htbl_item_t),
		    offsetof(htbl_item_t, bucket_node));
	htbl->key_sz = key_sz;
}

void
htbl_destroy(htbl_t *htbl)
{
	assert(htbl->num_items == 0);
	assert(htbl->buckets != NULL);
	for (size_t i = 0; i < htbl->mask + 1; i++)
		list_destroy(&htbl->buckets[i]);
	free(htbl->buckets);
	memset(htbl, 0, sizeof (*htbl));
}

void
htbl_empty(htbl_t *htbl)
{
	if (htbl->num_items == 0)
		return;
	for (size_t i = 0; i < htbl->mask + 1; i++) {
		while (!list_is_empty(&htbl->buckets[i])) {
			htbl_item_t *item = list_head(&htbl->buckets[i]);
			list_remove_head(&htbl->buckets[i]);
			free(item);
		}
	}
	htbl->num_items = 0;
}

size_t
htbl_count(const htbl_t *htbl)
{
	return (htbl->num_items);
}

void
htbl_add(htbl_t *htbl, void *key, void *value, int replace_ok)
{
	uint64_t h = XXH(key, htbl->key_sz) & htbl->mask;
	list_t *bucket = &htbl->buckets[h];
	htbl_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			assert(replace_ok != 0);
			item->value = value;
			return;
		}
	}
	item = malloc(sizeof (*item) + htbl->key_sz - 1);
	memcpy(item->key, key, htbl->key_sz);
	item->value = value;
	list_insert_head(bucket, item);
	htbl->num_items++;
}

void
htbl_remove(htbl_t *htbl, void *key, int nil_ok)
{
	uint64_t h = XXH(key, htbl->key_sz) & htbl->mask;
	list_t *bucket = &htbl->buckets[h];
	htbl_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			list_remove(bucket, item);
			free(item);
			htbl->num_items--;
			return;
		}
	}
	assert(nil_ok != 0);
}

void *
htbl_lookup(const htbl_t *htbl, void *key)
{
	uint64_t h = XXH(key, htbl->key_sz) & htbl->mask;
	list_t *bucket = &htbl->buckets[h];
	htbl_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0)
			return (item->value);
	}
	return (NULL);
}
