#ifndef _UNIQUE_ID_HPP_
#define _UNIQUE_ID_HPP_ 1

/** @file unique_id.cpp

    @brief Helper functions for universally unique identifiers and messages.
    This is a wrapper to avoid this problem:
    https://github.com/ros-geographic-info/unique_identifier/issues/7

    @author Jorge Santos

    @date Sep 26, 2014
 */

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <uuid_msgs/UniqueID.h>


namespace wcf
{
namespace uuid
{

/** @brief C++ namespace for unique_id helper functions.
 *
 *  Various ROS components use universally unique identifiers. This
 *  header provides functions for working with a common
 *  uuid_msgs/UniqueID message, and the boost uuid class.
 *
 *   - http://en.wikipedia.org/wiki/Uuid
 *   - http://tools.ietf.org/html/rfc4122.html
 *   - http://www.boost.org/doc/libs/1_42_0/libs/uuid/uuid.html
 *
 *  Programmers are free to create UUID objects using any approved RFC
 *  4122 method. The boost uuid interface supports them all.
 *
 *  Functions in this namespace provide simple APIs, not requiring
 *  detailed knowledge of RFC 4122 or the boost uuid interface.  ROS
 *  applications are likely to need either a random or a name-based
 *  UUID.
 *
 *   - fromRandom() generates a random UUID.
 *   - fromURL() generates a name-based UUID from a URL string.
 */

/** @brief Create UUID object from UniqueID message.
 *
 *  @param msg uuid_msgs/UniqueID message.
 *  @returns boost::uuids::uuid object.
 */
boost::uuids::uuid fromMsg(uuid_msgs::UniqueID const &msg);

/** @brief Generate a random UUID object.
 *
 *  @returns type 4 boost::uuids::uuid object.
 *
 *  Different calls to this function at any time or place will almost
 *  certainly generate different UUIDs. The method used is RFC 4122
 *  variant 4.
 */
boost::uuids::uuid fromRandom(void);

/** @brief Generate UUID from canonical hex string.
 *
 *  @param str string containing canonical hex representation
 *  @returns corresponding boost::uuids::uuid object.
 *
 *  @note This is not a general service for generating a UUID from an
 *  arbitrary character string. The fromURL() function will do that
 *  for any Uniform Resource Identifier.
 *
 *  The canonical hex string is a human-readable representation of a
 *  correctly-formatted sixteen-byte UUID. In addition to the dashes,
 *  it should contain exactly 32 hexadecimal digits.  The @a str can
 *  be any accepted by the boost uuid string generator, but that is
 *  not well-defined. The format produced by toHexString() works
 *  reliably: "01234567-89ab-cdef-0123-456789abcdef".
 *
 *  @warning Strings not accepted by boost may produce undefined
 *  results: perhaps throwing a @c std::runtime_error exception, or
 *  silently ignoring parts of the string.
 */
boost::uuids::uuid fromHexString(std::string const &str);

/** @brief Generate UUID from Uniform Resource Identifier.
 *
 *  @param url URL for identifier creation.
 *  @returns type 5 boost::uuids::uuid object.
 *
 *  Matching @a url strings must yield the same UUID. Different @a url
 *  strings will almost certainly generate different UUIDs. The method
 *  used is RFC 4122 variant 5, computing the SHA-1 hash of the @a
 *  url.
 *
 *  For any given @a url, this function returns the same UUID as the
 *  corresponding Python @c unique_id.fromURL() function.
 *
 *  For example, Open Street Map identifiers are encoded like this,
 *  with decimal representations of the integer OSM node, way, or
 *  relation identifiers appended to the URL:
 *
 *   - fromURL("http://openstreetmap.org/node/123456789")
 *   - fromURL("http://openstreetmap.org/way/6543210")
 *   - fromURL("http://openstreetmap.org/relation/999999")
 */
boost::uuids::uuid fromURL(std::string const &url);

/** @brief Create a UniqueID message from a UUID object.
 *
 *  @param uu boost::uuids::uuid object.
 *  @returns uuid_msgs/UniqueID message.
 */
uuid_msgs::UniqueID toMsg(boost::uuids::uuid const &uu);

/** @brief Get the canonical string representation for a boost UUID.
 *
 *  @param uu boost::uuids::uuid object.
 *  @returns canonical UUID hex string: "01234567-89ab-cdef-0123-456789abcdef".
 *
 *  A @c boost::uuids::uuid object yields the same representation via
 *  its @c << operator or @c to_string() function.
 */
std::string toHexString(boost::uuids::uuid const &uu);

/** @brief Get the canonical string representation for a UniqueID message.
 *
 *  @param msg uuid_msgs/UniqueID message.
 *  @returns canonical UUID hex string: "01234567-89ab-cdef-0123-456789abcdef".
 */
std::string toHexString(uuid_msgs::UniqueID const &msg);

} // namespace uuid
} // namespace wcf

#endif // _UNIQUE_ID_HPP_
