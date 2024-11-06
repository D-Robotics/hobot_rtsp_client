/**
 * Copyright (c) 1996-2008 Live Networks, Inc
 * All rights reserved.
 * @Author:
 * @Mail:
 */
/**********
This library is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the
Free Software Foundation; either version 2.1 of the License, or (at your
option) any later version. (See <http://www.gnu.org/copyleft/lesser.html>.)

This library is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License
along with this library; if not, write to the Free Software Foundation, Inc.,
59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
**********/
// Base64 encoding and decoding
// implementation

#include "rtspclient/Base64.h"

#include <string.h>

namespace NS_base64 {
unsigned char* base64Decode(char* in, unsigned& resultSize,
                            bool trimTrailingZeros = true);
// returns a newly allocated array - of size "resultSize" - that
// the caller is responsible for delete[]ing.

char* base64Encode(char const* orig, unsigned origLength);
// returns a 0-terminated string that
// the caller is responsible for delete[]ing.

char* strDup(char const* str) {
  if (str == NULL) return NULL;
  size_t len = strlen(str) + 1;
  char* copy = new char[len];

  if (copy != NULL) {
    memcpy(copy, str, len);
  }
  return copy;
}

char* strDupSize(char const* str) {
  if (str == NULL) return NULL;
  size_t len = strlen(str) + 1;
  char* copy = new char[len];

  return copy;
}

static char base64DecodeTable[256];

static void initBase64DecodeTable() {
  int i;
  for (i = 0; i < 256; ++i) base64DecodeTable[i] = static_cast<char>(0x80);
  // default value: invalid

  for (i = 'A'; i <= 'Z'; ++i) base64DecodeTable[i] = 0 + (i - 'A');
  for (i = 'a'; i <= 'z'; ++i) base64DecodeTable[i] = 26 + (i - 'a');
  for (i = '0'; i <= '9'; ++i) base64DecodeTable[i] = 52 + (i - '0');
  base64DecodeTable[(unsigned char)'+'] = 62;
  base64DecodeTable[(unsigned char)'/'] = 63;
  base64DecodeTable[(unsigned char)'='] = 0;
}

unsigned char* base64Decode(char* in, unsigned& resultSize,
                            bool trimTrailingZeros) {
  static bool haveInitedBase64DecodeTable = false;
  if (!haveInitedBase64DecodeTable) {
    initBase64DecodeTable();
    haveInitedBase64DecodeTable = true;
  }

  unsigned char* out =
      (unsigned char*)strDupSize(in);  // ensures we have enough space
  int k = 0;
  int const jMax = strlen(in) - 3;
  // in case "in" is not a multiple of 4 bytes (although it should be)
  for (int j = 0; j < jMax; j += 4) {
    char inTmp[4], outTmp[4];
    for (int i = 0; i < 4; ++i) {
      inTmp[i] = in[i + j];
      outTmp[i] = base64DecodeTable[(unsigned char)inTmp[i]];
      if ((outTmp[i] & 0x80) != 0) outTmp[i] = 0;  // pretend the input was 'A'
    }

    out[k++] = (outTmp[0] << 2) | (outTmp[1] >> 4);
    out[k++] = (outTmp[1] << 4) | (outTmp[2] >> 2);
    out[k++] = (outTmp[2] << 6) | outTmp[3];
  }

  if (trimTrailingZeros) {
    while (k > 0 && out[k - 1] == '\0') --k;
  }
  resultSize = k;
  unsigned char* result = new unsigned char[resultSize];
  memmove(result, out, resultSize);
  delete[] out;

  return result;
}

static const char base64Char[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

char* base64Encode(char const* origSigned, unsigned origLength) {
  unsigned char const* orig = (unsigned char const*)
      origSigned;  // in case any input bytes have the MSB set
  if (orig == NULL) return NULL;

  unsigned const numOrig24BitValues = origLength / 3;
  bool havePadding = origLength > numOrig24BitValues * 3;
  bool havePadding2 = origLength == numOrig24BitValues * 3 + 2;
  unsigned const numResultBytes = 4 * (numOrig24BitValues + havePadding);
  char* result = new char[numResultBytes + 1];  // allow for trailing '\0'

  // Map each full group of 3 input bytes into 4 output base-64 characters:
  unsigned i;
  for (i = 0; i < numOrig24BitValues; ++i) {
    result[4 * i + 0] = base64Char[(orig[3 * i] >> 2) & 0x3F];
    result[4 * i + 1] =
        base64Char[(((orig[3 * i] & 0x3) << 4) | (orig[3 * i + 1] >> 4)) &
                   0x3F];
    result[4 * i + 2] =
        base64Char[((orig[3 * i + 1] << 2) | (orig[3 * i + 2] >> 6)) & 0x3F];
    result[4 * i + 3] = base64Char[orig[3 * i + 2] & 0x3F];
  }

  // Now, take padding into account.  (Note: i == numOrig24BitValues)
  if (havePadding) {
    result[4 * i + 0] = base64Char[(orig[3 * i] >> 2) & 0x3F];
    if (havePadding2) {
      result[4 * i + 1] =
          base64Char[(((orig[3 * i] & 0x3) << 4) | (orig[3 * i + 1] >> 4)) &
                     0x3F];
      result[4 * i + 2] = base64Char[(orig[3 * i + 1] << 2) & 0x3F];
    } else {
      result[4 * i + 1] = base64Char[((orig[3 * i] & 0x3) << 4) & 0x3F];
      result[4 * i + 2] = '=';
    }
    result[4 * i + 3] = '=';
  }

  result[numResultBytes] = '\0';
  return result;
}
};  // namespace NS_base64

int base64::base64Decode(const char* p_pszDataIn, const int p_nInLen,
                         char* p_pszBufferOut, int& p_nOutLen,
                         bool p_bTrimTrailingZeros /* = true */) {
  static bool haveInitedBase64DecodeTable = false;
  if (!haveInitedBase64DecodeTable) {
    NS_base64::initBase64DecodeTable();
    haveInitedBase64DecodeTable = true;
  }

  if (p_nOutLen < p_nInLen || p_nInLen <= 0 || p_pszDataIn == NULL ||
      p_pszBufferOut == NULL)
    return -1;

  int k = 0;
  int const jMax = p_nInLen - 3;
  // in case "in" is not a multiple of 4 bytes (although it should be)
  for (int j = 0; j < jMax; j += 4) {
    char inTmp[4], outTmp[4];
    for (int i = 0; i < 4; ++i) {
      inTmp[i] = p_pszDataIn[i + j];
      outTmp[i] = NS_base64::base64DecodeTable[(unsigned char)inTmp[i]];
      if ((outTmp[i] & 0x80) != 0) outTmp[i] = 0;  // pretend the input was 'A'
    }

    p_pszBufferOut[k++] = (outTmp[0] << 2) | (outTmp[1] >> 4);
    p_pszBufferOut[k++] = (outTmp[1] << 4) | (outTmp[2] >> 2);
    p_pszBufferOut[k++] = (outTmp[2] << 6) | outTmp[3];
  }

  if (p_bTrimTrailingZeros) {
    while (k > 0 && p_pszBufferOut[k - 1] == '\0') --k;
  }

  p_nOutLen = k;
  return 0;
}

int base64::base64Encode(const char* p_pszDataIn, const int p_nInLen,
                         char* p_pszBufferOut, int& p_nOutLen) {
  if (p_pszBufferOut == NULL || p_pszDataIn == NULL || p_nOutLen < p_nInLen ||
      p_nInLen <= 0)
    return -1;

  unsigned char const* orig = (unsigned char const*)
      p_pszDataIn;  // in case any input bytes have the MSB set

  unsigned const numOrig24BitValues = p_nInLen / 3;
  bool havePadding = (unsigned)p_nInLen > numOrig24BitValues * 3;
  bool havePadding2 = (unsigned)p_nInLen == numOrig24BitValues * 3 + 2;
  // bool havePadding = (p_nInLen > numOrig24BitValues*3)?1:0;
  // bool havePadding2 = (p_nInLen == numOrig24BitValues*3 + 2)?1:0;
  unsigned const numResultBytes = 4 * (numOrig24BitValues + havePadding);

  // Map each full group of 3 input bytes into 4 output base-64 characters:
  unsigned i;
  for (i = 0; i < numOrig24BitValues; ++i) {
    p_pszBufferOut[4 * i + 0] =
        NS_base64::base64Char[(orig[3 * i] >> 2) & 0x3F];
    p_pszBufferOut[4 * i + 1] = NS_base64::base64Char
        [(((orig[3 * i] & 0x3) << 4) | (orig[3 * i + 1] >> 4)) & 0x3F];
    p_pszBufferOut[4 * i + 2] = NS_base64::base64Char[((orig[3 * i + 1] << 2) |
                                                       (orig[3 * i + 2] >> 6)) &
                                                      0x3F];
    p_pszBufferOut[4 * i + 3] = NS_base64::base64Char[orig[3 * i + 2] & 0x3F];
  }

  // Now, take padding into account.  (Note: i == numOrig24BitValues)
  if (havePadding) {
    p_pszBufferOut[4 * i + 0] =
        NS_base64::base64Char[(orig[3 * i] >> 2) & 0x3F];
    if (havePadding2) {
      p_pszBufferOut[4 * i + 1] = NS_base64::base64Char
          [(((orig[3 * i] & 0x3) << 4) | (orig[3 * i + 1] >> 4)) & 0x3F];
      p_pszBufferOut[4 * i + 2] =
          NS_base64::base64Char[(orig[3 * i + 1] << 2) & 0x3F];
    } else {
      p_pszBufferOut[4 * i + 1] =
          NS_base64::base64Char[((orig[3 * i] & 0x3) << 4) & 0x3F];
      p_pszBufferOut[4 * i + 2] = '=';
    }
    p_pszBufferOut[4 * i + 3] = '=';
  }

  p_nOutLen = numResultBytes;
  p_pszBufferOut[numResultBytes] = '\0';

  return 0;
}
