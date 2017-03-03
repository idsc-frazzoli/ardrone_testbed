/* -*- mode: C++ -*- */
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Jack O'Quin
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef _IMPL_UNIQUE_ID_H_
#define _IMPL_UNIQUE_ID_H_ 1

/** @file

    @brief Private implementation details for C++ unique_id interface.

    @author Jack O'Quin
 */

#include <boost/uuid/uuid_generators.hpp>

namespace unique_id
{

  /** @brief C++ namespace for private implementation details. */
  namespace impl
  {

    /* Instantiate boost string UUID generator. */
    static boost::uuids::string_generator genString;

    /* Instantiate boost random UUID generator. */
    static boost::uuids::random_generator genRandom;

    /* RFC 4122 namespace for URL identifiers. */
    static const std::string url_namespace =
      "6ba7b811-9dad-11d1-80b4-00c04fd430c8";

    /* RFC 4122 UUID for URL identifiers. */
    static const boost::uuids::uuid url_namespace_uuid =
      genString(url_namespace);

    /* Instantiate boost URL name UUID generator. */
    static boost::uuids::name_generator genURL(url_namespace_uuid);

  } // end namespace impl

} // end namespace unique_id

#endif // _IMPL_UNIQUE_ID_H_
