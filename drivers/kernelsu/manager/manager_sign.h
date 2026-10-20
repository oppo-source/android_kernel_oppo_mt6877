#ifndef MANAGER_SIGN_H
#define MANAGER_SIGN_H

// KittiSU/KittiSU
#define EXPECTED_SIZE_KITTISU 0x0390
#define EXPECTED_HASH_KITTISU "8a38ea45034a145e29234c4927743b088ef502441f8d24583a1a18f20e4c879a"

typedef struct {
    unsigned size;
    const char *sha256;
} apk_sign_key_t;

#endif /* MANAGER_SIGN_H */
