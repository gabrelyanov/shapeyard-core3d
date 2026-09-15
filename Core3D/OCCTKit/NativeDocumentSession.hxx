#pragma once

#include "OcctDocument.h"

namespace core3d {

//! Owns the native document independently of UIKit and graphics resources.
//! Access and destruction remain on the host's single writer (main thread).
//! One session belongs to one editor. Sharing live edit authority between
//! windows is intentionally unavailable until attachment/retirement is defined.
class NativeDocumentSession final {
public:
    NativeDocumentSession();
    ~NativeDocumentSession();
    NativeDocumentSession(const NativeDocumentSession&) = delete;
    NativeDocumentSession& operator=(const NativeDocumentSession&) = delete;
    NativeDocumentSession(NativeDocumentSession&&) = delete;
    NativeDocumentSession& operator=(NativeDocumentSession&&) = delete;

    const Handle(OcctDocument)& Document() const noexcept { return document_; }
    //! Idempotent terminal close, after native workers and presentations drain.
    void Close() noexcept;

private:
    Handle(OcctDocument) document_;
};

} // namespace core3d
