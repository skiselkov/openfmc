#ifndef	_OPENFMC_HTBL_H_
#define	_OPENFMC_HTBL_H_

#include <stdint.h>

#include "list.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	list_node_t	bucket_node;
	void		*value;
	uint8_t		key[1];	/* variable length, depends on htbl->key_sz */
} htbl_item_t;

typedef struct {
	size_t		mask;
	size_t		key_sz;
	list_t		*buckets;
	size_t		num_items;
} htbl_t;

void htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz);
void htbl_destroy(htbl_t *htbl);
void htbl_empty(htbl_t *htbl);
size_t htbl_count(const htbl_t *htbl);

void htbl_add(htbl_t *htbl, void *key, void *value, int replace_ok);
void htbl_remove(htbl_t *htbl, void *key, int nil_ok);
void *htbl_lookup(const htbl_t *htbl, void *key);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_HTBL_H_ */
