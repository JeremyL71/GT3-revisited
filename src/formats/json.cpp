#include "formats/json.h"

#include "base/bytes.h"

#include <cstdlib>
#include <cstring>

namespace gt3r::json {

const Value& Value::operator[](size_t i) const {
    static const Value null;
    return type_ == Type::Array && i < arr_.size() ? arr_[i] : null;
}

const Value& Value::operator[](std::string_view key) const {
    static const Value null;
    if (type_ != Type::Object) return null;
    auto it = obj_.find(std::string(key));
    return it == obj_.end() ? null : it->second;
}

struct Parser {
    std::string_view s;
    size_t i = 0;
    int depth = 0;

    [[noreturn]] void fail(const char* what) const {
        throw FormatError(std::string("JSON: ") + what + " at byte " + std::to_string(i));
    }
    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) ++i;
    }
    bool eat(char c) {
        ws();
        if (i < s.size() && s[i] == c) {
            ++i;
            return true;
        }
        return false;
    }
    void expect(char c) {
        if (!eat(c)) fail("unexpected character");
    }
    bool word(const char* w) {
        size_t n = std::strlen(w);
        if (s.substr(i, n) == w) {
            i += n;
            return true;
        }
        return false;
    }
    static void utf8(std::string& out, unsigned cp) {
        if (cp < 0x80) out += char(cp);
        else if (cp < 0x800) out += char(0xC0 | cp >> 6), out += char(0x80 | (cp & 0x3F));
        else if (cp < 0x10000) out += char(0xE0 | cp >> 12), out += char(0x80 | (cp >> 6 & 0x3F)), out += char(0x80 | (cp & 0x3F));
        else
            out += char(0xF0 | cp >> 18), out += char(0x80 | (cp >> 12 & 0x3F)), out += char(0x80 | (cp >> 6 & 0x3F)),
                out += char(0x80 | (cp & 0x3F));
    }
    unsigned hex4() {
        if (i + 4 > s.size()) fail("bad \\u escape");
        unsigned v = 0;
        for (int k = 0; k < 4; ++k) {
            char c = s[i++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= unsigned(c - '0');
            else if (c >= 'a' && c <= 'f') v |= unsigned(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= unsigned(c - 'A' + 10);
            else fail("bad \\u escape");
        }
        return v;
    }
    std::string string() {
        expect('"');
        std::string out;
        while (true) {
            if (i >= s.size()) fail("unterminated string");
            char c = s[i++];
            if (c == '"') break;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i >= s.size()) fail("unterminated string");
            char e = s[i++];
            switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                unsigned cp = hex4();
                if (cp >= 0xD800 && cp < 0xDC00 && s.substr(i, 2) == "\\u") {
                    i += 2;
                    unsigned lo = hex4();
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                }
                utf8(out, cp);
                break;
            }
            default: fail("bad escape");
            }
        }
        return out;
    }
    Value value() {
        if (++depth > 256) fail("nested too deeply");
        ws();
        if (i >= s.size()) fail("unexpected end");
        Value v;
        char c = s[i];
        if (c == '{') {
            ++i;
            v.type_ = Value::Type::Object;
            if (!eat('}')) {
                do {
                    ws();
                    std::string k = string();
                    expect(':');
                    v.obj_[k] = value();
                } while (eat(','));
                expect('}');
            }
        } else if (c == '[') {
            ++i;
            v.type_ = Value::Type::Array;
            if (!eat(']')) {
                do v.arr_.push_back(value());
                while (eat(','));
                expect(']');
            }
        } else if (c == '"') {
            v.type_ = Value::Type::String;
            v.str_ = string();
        } else if (word("true")) {
            v.type_ = Value::Type::Bool, v.b_ = true;
        } else if (word("false")) {
            v.type_ = Value::Type::Bool, v.b_ = false;
        } else if (word("null")) {
            v.type_ = Value::Type::Null;
        } else if (c == '-' || (c >= '0' && c <= '9')) {
            size_t start = i;
            while (i < s.size() && std::strchr("+-0123456789.eE", s[i])) ++i;
            std::string t(s.substr(start, i - start));
            char* end = nullptr;
            v.type_ = Value::Type::Number;
            v.num_ = std::strtod(t.c_str(), &end);
            if (!end || *end) fail("bad number");
        } else {
            fail("unexpected character");
        }
        --depth;
        return v;
    }
};

Value Value::parse(std::string_view text) {
    Parser p{text};
    Value v = p.value();
    p.ws();
    if (p.i != text.size()) p.fail("trailing data");
    return v;
}

} // namespace gt3r::json
