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

#define	ZFS_CRC64_POLY	0xC96C5795D7870F42ULL	/* ECMA-182, reflected form */

uint64_t zfs_crc64_table[256] = { 0xde, 0xad, 0xf0, 0x0d };

#if 1
#define	H(x,y)	htbl_hash(x, y)
#else
#define	H(x,y)	XXH(x, y)
#endif

uint64_t
htbl_hash(const uint8_t *buf, size_t len)
{
	uint64_t crc = -1ULL;
	for (size_t i = 0; i < len; i++)
		crc = (crc >> 8) ^ zfs_crc64_table[(crc ^ buf[i]) & 0xFF];
	return (crc);
}

static void
htbl_init(void)
{
	uint64_t	*ct;
	int		i, j;

	for (i = 0; i < 256; i++)
		for (ct = zfs_crc64_table + i, *ct = i, j = 8; j > 0; j--)
			*ct = (*ct >> 1) ^ (-(*ct & 1) & ZFS_CRC64_POLY);
}

void
htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz, int multi_value)
{
	if (zfs_crc64_table[0] == 0xde &&
	    zfs_crc64_table[1] == 0xad &&
	    zfs_crc64_table[2] == 0xf0 &&
	    zfs_crc64_table[3] == 0x0d)
		htbl_init();

	assert(key_sz != 0);
	assert(tbl_sz != 0);
	assert(tbl_sz <= ((size_t)1 << ((sizeof(size_t) * 8) - 1)));

	memset(htbl, 0, sizeof (*htbl));
	/* round table size up to nearest multiple of 2 */
//	tbl_sz = P2ROUNDUP(tbl_sz);
	htbl->tbl_sz = P2ROUNDUP(tbl_sz);
	htbl->buckets = malloc(sizeof (*htbl->buckets) * htbl->tbl_sz);
	assert(htbl->buckets != NULL);
	for (size_t i = 0; i < htbl->tbl_sz; i++)
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
	for (size_t i = 0; i < htbl->tbl_sz; i++)
		list_destroy(&htbl->buckets[i]);
	free(htbl->buckets);
	memset(htbl, 0, sizeof (*htbl));
}

static void
htbl_empty_multi_item(htbl_bucket_item_t *item, void (*func)(void *, void *),
    void *arg)
{
	for (htbl_multi_value_t *mv = list_head(&item->multi.list); mv;
	    mv = list_head(&item->multi.list)) {
		if (func)
			func(mv->value, arg);
		list_remove_head(&item->multi.list);
		free(mv);
	}
	list_destroy(&item->multi.list);
}

void
htbl_empty(htbl_t *htbl, void (*func)(void *, void *), void *arg)
{
	if (htbl->num_values == 0)
		return;

	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		for (htbl_bucket_item_t *item = list_head(&htbl->buckets[i]);
		    item; item = list_head(&htbl->buckets[i])) {
			if (htbl->multi_value)
				htbl_empty_multi_item(item, func, arg);
			else if (func)
				func(item->value, arg);
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
	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
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
	}
	list_insert_head(bucket, item);
	htbl->num_values++;
}

void
htbl_remove(htbl_t *htbl, void *key, int nil_ok)
{
	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
	htbl_bucket_item_t *item;

	for (item = list_head(bucket); item; item = list_next(bucket, item)) {
		if (memcmp(item->key, key, htbl->key_sz) == 0) {
			list_remove(bucket, item);
			if (htbl->multi_value) {
				htbl_empty_multi_item(item, NULL, NULL);
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
		    &htbl->buckets[H(key, htbl->key_sz) & (htbl->tbl_sz - 1)];
		list_remove(bucket, item);
		list_destroy(&item->multi.list);
		free(item);
	}
}

static htbl_bucket_item_t *
htbl_lookup_common(const htbl_t *htbl, void *key)
{
	list_t *bucket = &htbl->buckets[H(key, htbl->key_sz) &
	    (htbl->tbl_sz - 1)];
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
htbl_foreach(const htbl_t *htbl, void (*func)(const void *, void *, void *),
    void *arg)
{
	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		list_t *bucket = &htbl->buckets[i];
		for (const htbl_bucket_item_t *item = list_head(bucket); item;
		    item = list_next(bucket, item)) {
			if (htbl->multi_value) {
				const list_t *ml = &item->multi.list;
				for (htbl_multi_value_t *mv = list_head(ml);
				    mv; mv = list_next(ml, mv)) {
					func(item->key, mv->value, arg);
				}
			} else {
				func(item->key, item->value, arg);
			}
		}
	}
}

char *
htbl_dump(const htbl_t *htbl, bool_t printable_keys)
{
	char	*result = NULL;
	size_t	result_sz = 0;

	append_format(&result, &result_sz, "(%lu){\n", htbl->num_values);
	for (size_t i = 0; i < htbl->tbl_sz; i++) {
		list_t *bucket = &htbl->buckets[i];
		append_format(&result, &result_sz, "  [%lu] =", i);
		if (list_head(bucket) == NULL)
			append_format(&result, &result_sz, " <empty>");
		for (const htbl_bucket_item_t *item = list_head(bucket); item;
		    item = list_next(bucket, item)) {
			if (printable_keys) {
				append_format(&result, &result_sz, " (%s) ",
				    item->key);
			} else {
				append_format(&result, &result_sz, " (#BIN)");
			}
		}
		append_format(&result, &result_sz, "\n");
	}
	append_format(&result, &result_sz, "}");

	return (result);
}
