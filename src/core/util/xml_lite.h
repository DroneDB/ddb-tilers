// SPDX-License-Identifier: MPL-2.0
//
// A small XML reader for the documents embedded in raster files (GMLJP2 coverage descriptions,
// GDAL metadata boxes): elements, attributes and text, entity and character references decoded,
// CDATA kept as text; comments, processing instructions and the document type are skipped. Not a
// validating parser: it reads well-formed documents and refuses (nullopt) what it cannot read.
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ddbt::xml {

struct Element {
    /// The name without its namespace prefix ("gml:Point" -> "Point"), as GDAL's
    /// CPLStripXMLNamespace leaves it.
    std::string name;
    /// Attributes in document order, names without their namespace prefix.
    std::vector<std::pair<std::string, std::string>> attributes;
    /// The element's own text (the concatenation of its text and CDATA children).
    std::string text;
    std::vector<Element> children;

    /// The value of attribute `name`, or nullptr.
    const std::string* Attribute(std::string_view attributeName) const;
    /// The first child element named `name`, or nullptr.
    const Element* Child(std::string_view childName) const;
    /// The child element at `path` ("a.b.c", each step the first child of that name), or nullptr.
    const Element* Path(std::string_view path) const;
    /// The first element named `name` in a pre-order walk of this element and its descendants
    /// (GDAL's CPLSearchXMLNode), or nullptr.
    const Element* Find(std::string_view elementName) const;
};

/// The root element of `text`, or nullopt when it is not a well-formed document.
std::optional<Element> Parse(std::string_view text);

/// `text` with the characters XML reserves escaped (&, <, >; and " when `attribute`).
std::string Escape(std::string_view text, bool attribute);

}  // namespace ddbt::xml
