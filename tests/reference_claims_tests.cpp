#include <rpp_control/ros/reference_claims.hpp>
#include <gtest/gtest.h>

TEST(ReferenceClaims, one_owner_per_interface_even_when_identity_matches)
{
  rpp_control::ReferenceClaims claims;
  const auto token = claims.claim("navigation", "generator-1", "operator");
  ASSERT_TRUE(token);
  EXPECT_FALSE(claims.claim("navigation", "generator-1", "operator"));
  EXPECT_FALSE(claims.claim("navigation", "generator-1", "other"));
  EXPECT_TRUE(claims.authorizes("navigation", *token));
  EXPECT_FALSE(claims.authorizes("navigation", ""));
  EXPECT_FALSE(claims.authorizes("navigation", "different-token"));
  EXPECT_FALSE(claims.authorizes("another-interface", *token));
}

TEST(ReferenceClaims, updates_require_a_claim_and_matching_token)
{
  rpp_control::ReferenceClaims claims;
  EXPECT_FALSE(claims.authorizes("navigation", ""));
  const auto token = claims.claim("navigation", "generator-1", "operator");
  ASSERT_TRUE(token);
  EXPECT_FALSE(claims.authorizes("navigation", ""));
  EXPECT_TRUE(claims.authorizes("navigation", *token));
}

TEST(ReferenceClaims, release_revokes_the_token_without_releasing_other_interfaces)
{
  rpp_control::ReferenceClaims claims;
  const auto first = claims.claim("first", "generator-1", "operator");
  const auto second = claims.claim("second", "generator-2", "other");
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_EQ(claims.interface_for(*first), "first");
  claims.release("first");
  EXPECT_FALSE(claims.interface_for(*first));
  EXPECT_FALSE(claims.authorizes("first", *first));
  EXPECT_TRUE(claims.authorizes("second", *second));
  EXPECT_TRUE(claims.claim("first", "generator-1", "new-owner"));
}

TEST(ReferenceClaims, replacing_the_generator_revokes_its_claim)
{
  rpp_control::ReferenceClaims claims;
  const auto token = claims.claim("navigation", "generator-1", "operator");
  ASSERT_TRUE(token);
  claims.retain([](const auto &) {return "generator-1";});
  EXPECT_TRUE(claims.authorizes("navigation", *token));
  claims.retain([](const auto &) {return "generator-2";});
  EXPECT_FALSE(claims.authorizes("navigation", *token));
  EXPECT_TRUE(claims.claim("navigation", "generator-2", "new-owner"));
}
