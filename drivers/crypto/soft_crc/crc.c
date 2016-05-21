/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/endian.h>

#include <mutek/mem_alloc.h>
#include <device/request.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/crypto.h>

struct soft_crc_state_s
{
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_NOTABLE
  uint32_t poly;
#else
  const uint32_t *poly;
#endif
  uint32_t    crc;
  bool_t rev:1;
  bool_t inv:1;
};

ALWAYS_INLINE uint32_t bit_swap_32(uint32_t x)
{
  x = endian_swap32(x);
  x = ((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4);
  x = ((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2);
  x = ((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1);
  return x;
}

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_NOTABLE
# define CRC_POLY(r, n) ((r) ? bit_swap_32(0x##n) : 0x##n)
#else
# define CRC_POLY(r, n) poly_##n##_rev##r
#endif

DRIVER_PV(struct soft_crc_private_s
{
  struct dev_request_dlqueue_s queue;
});

enum soft_crc_variants_e
{
  SOFT_CRC32C,
  SOFT_CRC32,
  SOFT_CRC32_IEC13818,
  SOFT_CRC32_POSIX,
  SOFT_CRC32_ETSI,
  SOFT_CRC32_count,
};

static DEVCRYPTO_INFO(soft_crc_info)
{
  memset(info, 0, sizeof(*info));
  info->modes_mask = 1 << DEV_CRYPTO_MODE_HASH;
  info->cap |= DEV_CRYPTO_CAP_STATEFUL;
  info->state_size = sizeof(struct soft_crc_state_s);
  info->block_len = 0;

  static const char *names[SOFT_CRC32_count] = {
    "crc32c", "crc32", "crc32_iec13818", "crc32_posix", "crc32_etsi"
  };

  info->name = names[accessor->number];

  return 0;
}

#ifndef CONFIG_DRIVER_CRYPTO_SOFT_CRC_NOTABLE

# if defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_IEC13818) ||   \
     defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_POSIX) ||   \
     defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_ETSI)
static const uint32_t poly_04c11db7_rev0[256] = {
  0x00000000, 0xb71dc104, 0x6e3b8209, 0xd926430d, 0xdc760413, 0x6b6bc517, 0xb24d861a, 0x0550471e,
  0xb8ed0826, 0x0ff0c922, 0xd6d68a2f, 0x61cb4b2b, 0x649b0c35, 0xd386cd31, 0x0aa08e3c, 0xbdbd4f38,
  0x70db114c, 0xc7c6d048, 0x1ee09345, 0xa9fd5241, 0xacad155f, 0x1bb0d45b, 0xc2969756, 0x758b5652,
  0xc836196a, 0x7f2bd86e, 0xa60d9b63, 0x11105a67, 0x14401d79, 0xa35ddc7d, 0x7a7b9f70, 0xcd665e74,
  0xe0b62398, 0x57abe29c, 0x8e8da191, 0x39906095, 0x3cc0278b, 0x8bdde68f, 0x52fba582, 0xe5e66486,
  0x585b2bbe, 0xef46eaba, 0x3660a9b7, 0x817d68b3, 0x842d2fad, 0x3330eea9, 0xea16ada4, 0x5d0b6ca0,
  0x906d32d4, 0x2770f3d0, 0xfe56b0dd, 0x494b71d9, 0x4c1b36c7, 0xfb06f7c3, 0x2220b4ce, 0x953d75ca,
  0x28803af2, 0x9f9dfbf6, 0x46bbb8fb, 0xf1a679ff, 0xf4f63ee1, 0x43ebffe5, 0x9acdbce8, 0x2dd07dec,
  0x77708634, 0xc06d4730, 0x194b043d, 0xae56c539, 0xab068227, 0x1c1b4323, 0xc53d002e, 0x7220c12a,
  0xcf9d8e12, 0x78804f16, 0xa1a60c1b, 0x16bbcd1f, 0x13eb8a01, 0xa4f64b05, 0x7dd00808, 0xcacdc90c,
  0x07ab9778, 0xb0b6567c, 0x69901571, 0xde8dd475, 0xdbdd936b, 0x6cc0526f, 0xb5e61162, 0x02fbd066,
  0xbf469f5e, 0x085b5e5a, 0xd17d1d57, 0x6660dc53, 0x63309b4d, 0xd42d5a49, 0x0d0b1944, 0xba16d840,
  0x97c6a5ac, 0x20db64a8, 0xf9fd27a5, 0x4ee0e6a1, 0x4bb0a1bf, 0xfcad60bb, 0x258b23b6, 0x9296e2b2,
  0x2f2bad8a, 0x98366c8e, 0x41102f83, 0xf60dee87, 0xf35da999, 0x4440689d, 0x9d662b90, 0x2a7bea94,
  0xe71db4e0, 0x500075e4, 0x892636e9, 0x3e3bf7ed, 0x3b6bb0f3, 0x8c7671f7, 0x555032fa, 0xe24df3fe,
  0x5ff0bcc6, 0xe8ed7dc2, 0x31cb3ecf, 0x86d6ffcb, 0x8386b8d5, 0x349b79d1, 0xedbd3adc, 0x5aa0fbd8,
  0xeee00c69, 0x59fdcd6d, 0x80db8e60, 0x37c64f64, 0x3296087a, 0x858bc97e, 0x5cad8a73, 0xebb04b77,
  0x560d044f, 0xe110c54b, 0x38368646, 0x8f2b4742, 0x8a7b005c, 0x3d66c158, 0xe4408255, 0x535d4351,
  0x9e3b1d25, 0x2926dc21, 0xf0009f2c, 0x471d5e28, 0x424d1936, 0xf550d832, 0x2c769b3f, 0x9b6b5a3b,
  0x26d61503, 0x91cbd407, 0x48ed970a, 0xfff0560e, 0xfaa01110, 0x4dbdd014, 0x949b9319, 0x2386521d,
  0x0e562ff1, 0xb94beef5, 0x606dadf8, 0xd7706cfc, 0xd2202be2, 0x653deae6, 0xbc1ba9eb, 0x0b0668ef,
  0xb6bb27d7, 0x01a6e6d3, 0xd880a5de, 0x6f9d64da, 0x6acd23c4, 0xddd0e2c0, 0x04f6a1cd, 0xb3eb60c9,
  0x7e8d3ebd, 0xc990ffb9, 0x10b6bcb4, 0xa7ab7db0, 0xa2fb3aae, 0x15e6fbaa, 0xccc0b8a7, 0x7bdd79a3,
  0xc660369b, 0x717df79f, 0xa85bb492, 0x1f467596, 0x1a163288, 0xad0bf38c, 0x742db081, 0xc3307185,
  0x99908a5d, 0x2e8d4b59, 0xf7ab0854, 0x40b6c950, 0x45e68e4e, 0xf2fb4f4a, 0x2bdd0c47, 0x9cc0cd43,
  0x217d827b, 0x9660437f, 0x4f460072, 0xf85bc176, 0xfd0b8668, 0x4a16476c, 0x93300461, 0x242dc565,
  0xe94b9b11, 0x5e565a15, 0x87701918, 0x306dd81c, 0x353d9f02, 0x82205e06, 0x5b061d0b, 0xec1bdc0f,
  0x51a69337, 0xe6bb5233, 0x3f9d113e, 0x8880d03a, 0x8dd09724, 0x3acd5620, 0xe3eb152d, 0x54f6d429,
  0x7926a9c5, 0xce3b68c1, 0x171d2bcc, 0xa000eac8, 0xa550add6, 0x124d6cd2, 0xcb6b2fdf, 0x7c76eedb,
  0xc1cba1e3, 0x76d660e7, 0xaff023ea, 0x18ede2ee, 0x1dbda5f0, 0xaaa064f4, 0x738627f9, 0xc49be6fd,
  0x09fdb889, 0xbee0798d, 0x67c63a80, 0xd0dbfb84, 0xd58bbc9a, 0x62967d9e, 0xbbb03e93, 0x0cadff97,
  0xb110b0af, 0x060d71ab, 0xdf2b32a6, 0x6836f3a2, 0x6d66b4bc, 0xda7b75b8, 0x035d36b5, 0xb440f7b1,
};
# endif

# if defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32)
static const uint32_t poly_04c11db7_rev1[256] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
  0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
  0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
  0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
  0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
  0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
  0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
  0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
  0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
  0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
  0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
  0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
  0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
  0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
  0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
  0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
  0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
  0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
  0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
  0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
  0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
  0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d,
};
# endif

# if defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32C)
static const uint32_t poly_1edc6f41_rev1[256] = {
  0x00000000, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c, 0x26a1e7e8, 0xd4ca64eb,
  0x8ad958cf, 0x78b2dbcc, 0x6be22838, 0x9989ab3b, 0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24,
  0x105ec76f, 0xe235446c, 0xf165b798, 0x030e349b, 0xd7c45070, 0x25afd373, 0x36ff2087, 0xc494a384,
  0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc, 0xbc267848, 0x4e4dfb4b,
  0x20bd8ede, 0xd2d60ddd, 0xc186fe29, 0x33ed7d2a, 0xe72719c1, 0x154c9ac2, 0x061c6936, 0xf477ea35,
  0xaa64d611, 0x580f5512, 0x4b5fa6e6, 0xb93425e5, 0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa,
  0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x05125dad, 0x1642ae59, 0xe4292d5a,
  0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a, 0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595,
  0x417b1dbc, 0xb3109ebf, 0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957,
  0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0x0c38d26c, 0xfe53516f, 0xed03a29b, 0x1f682198,
  0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927, 0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38,
  0xdbfc821c, 0x2997011f, 0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0x0f36e6f7,
  0x61c69362, 0x93ad1061, 0x80fde395, 0x72966096, 0xa65c047d, 0x5437877e, 0x4767748a, 0xb50cf789,
  0xeb1fcbad, 0x197448ae, 0x0a24bb5a, 0xf84f3859, 0x2c855cb2, 0xdeeedfb1, 0xcdbe2c45, 0x3fd5af46,
  0x7198540d, 0x83f3d70e, 0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6,
  0xfb410cc2, 0x092a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de, 0xdde0eb2a, 0x2f8b6829,
  0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c, 0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93,
  0x082f63b7, 0xfa44e0b4, 0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f, 0xdce5075c,
  0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b, 0xb4091bff, 0x466298fc,
  0x1871a4d8, 0xea1a27db, 0xf94ad42f, 0x0b21572c, 0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033,
  0xa24bb5a6, 0x502036a5, 0x4370c551, 0xb11b4652, 0x65d122b9, 0x97baa1ba, 0x84ea524e, 0x7681d14d,
  0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975, 0x0e330a81, 0xfc588982,
  0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d, 0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622,
  0x38cc2a06, 0xcaa7a905, 0xd9f75af1, 0x2b9cd9f2, 0xff56bd19, 0x0d3d3e1a, 0x1e6dcdee, 0xec064eed,
  0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x0417b1db, 0xf67c32d8, 0xe52cc12c, 0x1747422f,
  0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff, 0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0,
  0xd3d3e1ab, 0x21b862a8, 0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x07198540,
  0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90, 0x9e902e7b, 0x6cfbad78, 0x7fab5e8c, 0x8dc0dd8f,
  0xe330a81a, 0x115b2b19, 0x020bd8ed, 0xf0605bee, 0x24aa3f05, 0xd6c1bc06, 0xc5914ff2, 0x37faccf1,
  0x69e9f0d5, 0x9b8273d6, 0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e,
  0xf36e6f75, 0x0105ec76, 0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69, 0xd5cf889d, 0x27a40b9e,
  0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e, 0xbe2da0a5, 0x4c4623a6, 0x5f16d052, 0xad7d5351
};
# endif

#endif

static void soft_crc_update(struct soft_crc_state_s *st,
                            const uint8_t * __restrict__ data, size_t len)
{
  uint32_t crc = st->crc;
  size_t i;

#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_NOTABLE

  uint32_t poly = st->poly;
  for (i = 0; i < len; i++)
    {
# if defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32) ||   \
     defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32C)
      if (st->rev)
        {
          uint32_t w = data[i];
          size_t j;

          for (j = 0; j < 8; j++)
            {
              crc = (crc >> 1) ^ (poly & ~(((w ^ crc) & 1) - 1));
              w >>= 1;
            }
        }
# endif

# if defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_IEC13818) ||   \
     defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_POSIX) ||   \
     defined(CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_ETSI)
      if (!st->rev)
        {
          uint32_t w = data[i] << 24;
          size_t j;

          for (j = 0; j < 8; j++)
            {
              crc = (crc << 1) ^ (poly & ~(((w ^ crc) >> 31) - 1));
              w <<= 1;
            }
        }
#endif
    }

#else

  const uint32_t *poly = st->poly;

  for (i = 0; i < len; i++)
    crc = (crc >> 8) ^ poly[data[i] ^ (uint8_t)crc];

#endif

  st->crc = crc;
}

static DEV_REQUEST_DELAYED_FUNC(soft_crc_process)
{
  struct device_s *dev = accessor->dev;
  struct soft_crc_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq = dev_crypto_rq_s_cast(rq_);
  struct dev_crypto_context_s *ctx = rq->ctx;
  struct soft_crc_state_s *st = ctx->state_data;

  rq->err = -ENOTSUP;

  if (ctx->mode != DEV_CRYPTO_MODE_HASH)
    goto pop;

  if (rq->op & DEV_CRYPTO_INIT)
    {
      if (rq->op & DEV_CRYPTO_FINALIZE)
        st = alloca(sizeof(*st));

      switch (accessor->number)
        {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32C
        case SOFT_CRC32C:
          st->crc = 0xffffffff;
          st->rev = 1;
          st->inv = 1;
          st->poly = CRC_POLY(1, 1edc6f41);
          break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32
        case SOFT_CRC32:
          st->crc = 0xffffffff;
          st->rev = 1;
          st->inv = 1;
          st->poly = CRC_POLY(1, 04c11db7);
          break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_IEC13818
        case SOFT_CRC32_IEC13818:
          st->crc = 0xffffffff;
          st->rev = 0;
          st->inv = 0;
          st->poly = CRC_POLY(0, 04c11db7);
          break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_POSIX
        case SOFT_CRC32_POSIX:
          st->crc = 0x00000000;
          st->rev = 0;
          st->inv = 1;
          st->poly = CRC_POLY(0, 04c11db7);
          break;
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_ETSI
        case SOFT_CRC32_ETSI:
          st->crc = 0xffffffff;
          st->rev = 0;
          st->inv = 1;
          st->poly = CRC_POLY(0, 04c11db7);
          break;
#endif
        default:
          UNREACHABLE();
        }
    }

  soft_crc_update(st, rq->ad, rq->ad_len);

  if (rq->op & DEV_CRYPTO_FINALIZE)
    {
      rq->err = -EINVAL;

      switch (rq->len)
        {
        case 4: {
          uint32_t x = st->crc;
          if (st->inv)
            x ^= 0xffffffff;
#ifndef CONFIG_DRIVER_CRYPTO_SOFT_CRC_NOTABLE
          if (!st->rev)
            endian_le32_na_store(rq->out, x);
          else
#endif
            endian_be32_na_store(rq->out, x);
          break;
        }
        default:
          goto pop;
        }
    }

  rq->err = 0;

 pop:
  dev_request_delayed_end(&pv->queue, rq_);
}

static DEVCRYPTO_REQUEST(soft_crc_request)
{
  struct device_s *dev = accessor->dev;
  struct soft_crc_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_crypto_s_base(accessor),
                           &pv->queue, dev_crypto_rq_s_base(rq), 0);
}

static DEV_USE(soft_crc_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      switch (accessor->number)
        {
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32C
        case SOFT_CRC32C:
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32
        case SOFT_CRC32:
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_IEC13818
        case SOFT_CRC32_IEC13818:
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_POSIX
        case SOFT_CRC32_POSIX:
#endif
#ifdef CONFIG_DRIVER_CRYPTO_SOFT_CRC_CRC32_ETSI
        case SOFT_CRC32_ETSI:
#endif
          return 0;
        default:
          return -ENOTSUP;
        }
    case DEV_USE_PUT_ACCESSOR:
    case DEV_USE_START:
    case DEV_USE_STOP:
      return 0;
    case DEV_USE_LAST_NUMBER:
      accessor->number = SOFT_CRC32_count - 1;
      return 0;
    default:
      return -ENOTSUP;
    }
}


static DEV_INIT(soft_crc_init)
{
  struct soft_crc_private_s *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_delayed_init(&pv->queue, &soft_crc_process);

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soft_crc_cleanup)
{
  struct soft_crc_private_s  *pv = dev->drv_pv;

  if (!dev_request_delayed_isidle(&pv->queue))
    return -EBUSY;

  dev_request_delayed_cleanup(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(soft_crc_drv, 0, "Software CRC hash", soft_crc,
               DRIVER_CRYPTO_METHODS(soft_crc));

DRIVER_REGISTER(soft_crc_drv);

DEV_DECLARE_STATIC(soft_crc_dev, "crc_soft", 0, soft_crc_drv);

