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
htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz, int multi_value)
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
		list_create(&htbl->buckets[i], sizeof (htbl_bucket_item_t),
		    offsetof(htbl_bucket_item_t, bucket_node));
	htbl->key_sz = key_sz;
	htbl->multi_value = multi_value;
}

void
htbl_destroy(htbl_t *htbl)
{
	assert(htbl->num_values == 0);
	assert(htbl->buckets != NULL);
	for (size_t i = 0; i < htbl->mask + 1; i++)
		list_destroy(&htbl->buckets[i]);
	free(htbl->buckets);
	memset(htbl, 0, sizeof (*htbl));
}

static void
htbl_empty_multi_item(htbl_bucket_item_t *item)
{
	for (htbl_multi_value_t *mv = list_head(&item->multi.list); mv;
	    mv = list_head(&item->multi.list)) {
		list_remove_head(&item->multi.list);
		free(mv);
	}
	list_destroy(&item->multi.list);
}

void
htbl_empty(htbl_t *htbl)
{
	if (htbl->num_values == 0)
		return;

	for (size_t i = 0; i < htbl->mask + 1; i++) {
		for (htbl_bucket_item_t *item = list_head(&htbl->buckets[i]);
		    item; item = list_head(&htbl->buckets[i])) {
			if (htbl->multi_value)
				htbl_empty_multi_item(item);
			list_remove_head(&htbl->buckets[i]);
			free(item);
		}
	}
	htbl->num_values = 0;
}

size_t
htbl_count(const htbl_t *htbl)
{
	return (htbl->num_values);
}

void
htbl_set(htbl_t *htbl, void *key, void *value)
{
	list_t *bucket = &htbl->buckets[XXH(key, htbl->key_sz) & htbl->mask];
	htbl_bucket_item_t *item;

	assert(key != NULL);
	assert(value != NULL);
	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			if (htbl->multi_value) {
				htbl_multi_value_t *mv = malloc(sizeof (*mv));
				mv->value = value;
				mv->item = item;
				list_insert_head(&item->multi.list, mv);
				item->multi.num++;
				htbl->num_values++;
			} else {
				item->value = value;
			}
			return;
		}
	}
	item = malloc(sizeof (*item) + htbl->key_sz - 1);
	memcpy(item->key, key, htbl->key_sz);
	if (htbl->multi_value) {
		htbl_multi_value_t *mv = malloc(sizeof (*mv));
		mv->value = value;
		mv->item = item;
		list_create(&item->multi.list, sizeof (htbl_multi_value_t),
		    offsetof(htbl_multi_value_t, node));
		list_insert_head(&item->multi.list, mv);
		item->multi.num = 1;
	} else {
		item->value = value;
		list_insert_head(bucket, item);
	}
	htbl->num_values++;
}

void
htbl_remove(htbl_t *htbl, void *key, int nil_ok)
{
	list_t *bucket = &htbl->buckets[XXH(key, htbl->key_sz) & htbl->mask];
	htbl_bucket_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			list_remove(bucket, item);
			if (htbl->multi_value) {
				htbl_empty_multi_item(item);
				assert(htbl->num_values >= item->multi.num);
				htbl->num_values -= item->multi.num;
			} else {
				free(item);
				assert(htbl->num_values != 0);
				htbl->num_values--;
			}
			return;
		}
	}
	assert(nil_ok != 0);
}

void htbl_remove_multi(htbl_t *htbl, void *key, void *list_item)
{
	htbl_multi_value_t *mv = list_item;
	htbl_bucket_item_t *item = mv->item;

	assert(htbl->multi_value != 0);
	assert(key != NULL);
	assert(item != NULL);
	assert(item->multi.num != 0);
	assert(htbl->num_values != 0);

	list_remove(&item->multi.list, mv);
	item->multi.num--;
	htbl->num_values--;
	free(mv);
	if (item->multi.num == 0) {
		list_t *bucket =
		    &htbl->buckets[XXH(key, htbl->key_sz) & htbl->mask];
		list_remove(bucket, item);
		list_destroy(&item->multi.list);
		free(item);
	}
}

static htbl_bucket_item_t *
htbl_lookup_common(const htbl_t *htbl, void *key)
{
	list_t *bucket = &htbl->buckets[XXH(key, htbl->key_sz) & htbl->mask];
	htbl_bucket_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0)
			return (item);
	}
	return (NULL);
}

void *
htbl_lookup(const htbl_t *htbl, void *key)
{
	htbl_bucket_item_t *item;
	assert(htbl->multi_value == 0);
	item = htbl_lookup_common(htbl, key);
	return (item != NULL ? item->value : NULL);
}

const list_t *
htbl_lookup_multi(const htbl_t *htbl, void *key)
{
	htbl_bucket_item_t *item;
	assert(htbl->multi_value != 0);
	item = htbl_lookup_common(htbl, key);
	return (item != NULL ? &item->multi.list : NULL);
}

void
htbl_foreach(const htbl_t *htbl, void (*func)(void *, void *), void *arg)
{
	for (size_t i = 0; i < htbl->mask + 1; i++) {
		list_t *bucket = &htbl->buckets[i];
		for (const htbl_bucket_item_t *item = list_head(bucket); item;
		    item = list_next(bucket, item)) {
			if (htbl->multi_value) {
				const list_t *ml = &item->multi.list;
				for (htbl_multi_value_t *mv = list_head(ml);
				    mv; mv = list_next(ml, mv)) {
					func(mv->value, arg);
				}
			} else {
				func(item->value, arg);
			}
		}
	}
}
