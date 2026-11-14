// SPDX-License-Identifier: MPL-2.0
#include <gtest/gtest.h>

#include "core/util/xml_lite.h"

namespace ddbt::xml {
namespace {

TEST(XmlLite, ElementsAttributesTextAndNamespaces) {
    const auto doc = Parse(R"(<?xml version="1.0"?>
<!-- a comment --><!DOCTYPE x [<!ENTITY y "z">]>
<gml:FeatureCollection xmlns:gml="http://www.opengis.net/gml">
  <gml:boundedBy><gml:Envelope srsName="urn:ogc:def:crs:EPSG::4326"/></gml:boundedBy>
  <gml:RectifiedGrid gml:id="g"><gml:offsetVector>1 0</gml:offsetVector><gml:offsetVector>0 -1</gml:offsetVector>
    <note><![CDATA[a<b]]> &amp; &#x41;&#66;&lt;</note><?pi skipped?>
  </gml:RectifiedGrid>
</gml:FeatureCollection>)");
    ASSERT_TRUE(doc.has_value());
    EXPECT_EQ(doc->name, "FeatureCollection");
    const Element* env = doc->Path("boundedBy.Envelope");
    ASSERT_NE(env, nullptr);
    ASSERT_NE(env->Attribute("srsName"), nullptr);
    EXPECT_EQ(*env->Attribute("srsName"), "urn:ogc:def:crs:EPSG::4326");
    const Element* rg = doc->Find("RectifiedGrid");
    ASSERT_NE(rg, nullptr);
    ASSERT_NE(rg->Attribute("id"), nullptr);  // the prefix of an attribute is stripped too
    ASSERT_EQ(rg->children.size(), 3u);
    EXPECT_EQ(rg->children[1].text, "0 -1");
    EXPECT_EQ(rg->Child("note")->text, "a<b & AB<");
    EXPECT_EQ(doc->Find("missing"), nullptr);
}

TEST(XmlLite, MalformedDocumentsAreRefused) {
    EXPECT_FALSE(Parse("").has_value());
    EXPECT_FALSE(Parse("text only").has_value());
    EXPECT_FALSE(Parse("<a><b></a>").has_value());
    EXPECT_FALSE(Parse("<a x=1/>").has_value());
    EXPECT_FALSE(Parse("<a>unterminated").has_value());
    std::string deep;
    for (int i = 0; i < 1000; ++i) deep += "<a>";
    EXPECT_FALSE(Parse(deep).has_value());
}

TEST(XmlLite, Escape) {
    EXPECT_EQ(Escape("a<b>&\"c\"", false), "a&lt;b&gt;&amp;\"c\"");
    EXPECT_EQ(Escape("a<b>&\"c\"", true), "a&lt;b&gt;&amp;&quot;c&quot;");
}

}  // namespace
}  // namespace ddbt::xml
