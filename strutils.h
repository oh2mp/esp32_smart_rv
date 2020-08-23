/* String utils */

#ifndef STRUTILS_H
#define STRUTILS_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#ifdef __cplusplus
extern "C" {
#endif

struct url_info {
    char *scheme;
    char *hostn;
    int port;
    char *path;
};

void split_url(struct url_info *ui, char *url);

/* split_url() - split url to its parts and store in a struct

Usage:

struct url_info urlp;
char url[] = "https://foo/bar";

parse_url(&urlp, url);

// now urlp.scheme contains "https", urlp.host contains "foo"
// urlp.path contains "/bar" and urlp.port contains int 443.

*/

/* ------------------------------------------------------------------------ */
// convert a hex string such as "A489B1" into an array like [0xA4, 0x89, 0xB1].

void hexToBytes(char *byteArray, const char *hexString);

// simple base64 encode
char b64_encode(const char *in, char *out, size_t len);

// returns the index of char c in string s
int strpos(const char *s, char c);

// in-place trim whitespace from right
void trimr(char *str);

// in-place utf8 to latin1 conversion
void utf8ToLatin(char *s);

#ifdef __cplusplus
}
#endif
#endif
