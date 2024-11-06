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
// C++ header

#ifndef _BASE64_HH
#define _BASE64_HH

class base64 {
 public:
  base64();
  virtual ~base64();

 public:
  static int base64Decode(const char* p_pszDataIn, const int p_nInLen,
                          char* p_pszBufferOut, int& p_nOutLen,
                          bool p_bTrimTrailingZeros = true);

  static int base64Encode(const char* p_pszDataIn, const int p_nInLen,
                          char* p_pszBufferOut, int& p_nOutLen);
};

#endif
