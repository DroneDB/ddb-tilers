// SPDX-License-Identifier: MPL-2.0
#pragma once

// Error handling of the core.
//
// Policy:
//  - Expected failures (I/O errors, corrupt or unsupported input, integer overflow while parsing,
//    memory budget exhausted, cancellation) are returned as Status / Expected<T>. They are cheap:
//    an OK Status is one null pointer.
//  - Exceptions are used only where an error must cross user code that cannot return a Status:
//    a task body run by a TaskGroup and a cache loader. Both are rethrown to the waiting thread.
//    std::bad_alloc is not caught inside the core.
//  - No exception crosses the C API: every entry point catches and maps to an error code.

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

namespace ddbt {

enum class StatusCode : uint8_t {
    kOk = 0,
    kInvalidArgument,
    kOutOfRange,        // offset / size past the end of a source, short read
    kNotFound,          // missing file or key
    kIoError,           // operating system I/O failure (errno / GetLastError in the message)
    kOverflow,          // integer overflow while computing sizes / offsets
    kCorrupt,           // malformed input data
    kUnsupported,       // valid input we do not handle
    kResourceExhausted, // memory budget or another limit reached
    kCancelled,
    kInternal,
};

/// Name of a code ("IoError", ...).
const char* StatusCodeName(StatusCode code) noexcept;

/// Result of an operation that returns nothing. OK is a null pointer (no allocation); an error
/// owns its code and message.
class [[nodiscard]] Status {
public:
    Status() noexcept = default;
    Status(StatusCode code, std::string message);

    Status(const Status& other) : rep_(other.rep_ ? std::make_unique<Rep>(*other.rep_) : nullptr) {}
    Status& operator=(const Status& other) {
        if (this != &other) rep_ = other.rep_ ? std::make_unique<Rep>(*other.rep_) : nullptr;
        return *this;
    }
    Status(Status&&) noexcept = default;
    Status& operator=(Status&&) noexcept = default;
    ~Status() = default;

    static Status Ok() noexcept { return Status(); }

    bool ok() const noexcept { return rep_ == nullptr; }
    explicit operator bool() const noexcept { return ok(); }
    StatusCode code() const noexcept { return rep_ ? rep_->code : StatusCode::kOk; }
    /// Message only (empty when OK).
    std::string_view message() const noexcept { return rep_ ? std::string_view(rep_->message) : std::string_view(); }
    /// "IoError: message" (or "OK").
    std::string ToString() const;

    /// Returns a copy whose message is prefixed with `context: `.
    Status WithContext(std::string_view context) const;

    /// Explicitly discards a Status (documents an intentional ignore).
    void IgnoreError() const noexcept {}

private:
    struct Rep {
        StatusCode code;
        std::string message;
    };
    std::unique_ptr<Rep> rep_;
};

// Shorthands
Status InvalidArgument(std::string message);
Status OutOfRange(std::string message);
Status NotFound(std::string message);
Status IoError(std::string message);
Status OverflowError(std::string message);
Status Corrupt(std::string message);
Status Unsupported(std::string message);
Status ResourceExhausted(std::string message);
Status Cancelled(std::string message);
Status InternalError(std::string message);

/// I/O error from an OS error number: "<what>: <strerror> (errno N)". ENOENT maps to kNotFound.
Status ErrnoStatus(std::string_view what, int err);
#ifdef _WIN32
/// I/O error from a Win32 error code (GetLastError()). ERROR_FILE_NOT_FOUND / PATH_NOT_FOUND map
/// to kNotFound.
Status Win32Status(std::string_view what, unsigned long err);
#endif

/// A value or an error Status. Construct from a T (success) or from a non-OK Status.
template <typename T>
class [[nodiscard]] Expected {
    static_assert(!std::is_same_v<std::remove_cv_t<T>, Status>, "Expected<Status> is meaningless");
    static_assert(!std::is_reference_v<T>, "Expected<T&> is not supported");

public:
    Expected(const T& value) : v_(std::in_place_index<0>, value) {}                    // NOLINT
    Expected(T&& value) noexcept(std::is_nothrow_move_constructible_v<T>)              // NOLINT
        : v_(std::in_place_index<0>, std::move(value)) {}
    template <typename U, typename = std::enable_if_t<std::is_constructible_v<T, U&&> &&
                                                      !std::is_same_v<std::decay_t<U>, T> &&
                                                      !std::is_same_v<std::decay_t<U>, Status> &&
                                                      !std::is_same_v<std::decay_t<U>, Expected>>>
    Expected(U&& value) : v_(std::in_place_index<0>, std::forward<U>(value)) {}       // NOLINT
    /// `status` must not be OK (an OK status becomes an Internal error).
    Expected(Status status) : v_(std::in_place_index<1>, FixOk(std::move(status))) {}  // NOLINT

    bool ok() const noexcept { return v_.index() == 0; }
    explicit operator bool() const noexcept { return ok(); }

    /// The error (OK when a value is held).
    Status status() const { return ok() ? Status() : std::get<1>(v_); }
    StatusCode code() const noexcept { return ok() ? StatusCode::kOk : std::get<1>(v_).code(); }

    // Precondition: ok() (otherwise std::bad_variant_access is thrown).
    T& value() & { return std::get<0>(v_); }
    const T& value() const& { return std::get<0>(v_); }
    T&& value() && { return std::get<0>(std::move(v_)); }
    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }
    T&& operator*() && { return std::move(*this).value(); }
    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }

    template <typename U>
    T value_or(U&& fallback) const& {
        return ok() ? value() : static_cast<T>(std::forward<U>(fallback));
    }

    /// Moves the error out (precondition: !ok()).
    Status TakeStatus() && { return std::get<1>(std::move(v_)); }

private:
    static Status FixOk(Status s) {
        if (s.ok()) return Status(StatusCode::kInternal, "Expected constructed from an OK status");
        return s;
    }
    std::variant<T, Status> v_;
};

}  // namespace ddbt

// Propagation helpers. DDBT_TRY(expr): returns the Status of `expr` if it is not OK (works for
// Status and Expected<T>). DDBT_ASSIGN_OR_RETURN(lhs, expr): `lhs = *expr` or return its error.
#define DDBT_STATUS_CONCAT_INNER(a, b) a##b
#define DDBT_STATUS_CONCAT(a, b) DDBT_STATUS_CONCAT_INNER(a, b)

#define DDBT_TRY(expr)                                                                      \
    do {                                                                                    \
        auto&& ddbt_try_result_ = (expr);                                                   \
        if (!ddbt_try_result_.ok()) return ::ddbt::detail::ToStatus(std::move(ddbt_try_result_)); \
    } while (false)

#define DDBT_ASSIGN_OR_RETURN_IMPL(tmp, lhs, expr)                \
    auto tmp = (expr);                                            \
    if (!tmp.ok()) return std::move(tmp).TakeStatus();            \
    lhs = std::move(tmp).value()

#define DDBT_ASSIGN_OR_RETURN(lhs, expr) \
    DDBT_ASSIGN_OR_RETURN_IMPL(DDBT_STATUS_CONCAT(ddbt_assign_tmp_, __LINE__), lhs, expr)

namespace ddbt::detail {
inline Status ToStatus(Status&& s) { return std::move(s); }
inline Status ToStatus(Status& s) { return std::move(s); }
template <typename T>
Status ToStatus(Expected<T>&& e) { return std::move(e).TakeStatus(); }
template <typename T>
Status ToStatus(Expected<T>& e) { return std::move(e).TakeStatus(); }
}  // namespace ddbt::detail
