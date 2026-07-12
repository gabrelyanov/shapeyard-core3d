#import "Core3DNativeImportOperation+Private.h"

#include "../OCCTKit/Core3DSTEPExchangeLock.h"
#include "../Common/Core3DMobileResourceLimits.h"
#include "../OCCTKit/OcctDocument.h"

#include <IFSelect_ReturnStatus.hxx>
#include <Interface_CheckIterator.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPConstruct_ExternRefs.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StepData_ConfParameters.hxx>
#include <StepData_StepModel.hxx>
#include <TDF_LabelMap.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_ChildIterator.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_MapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_MaterialTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <XSControl_WorkSession.hxx>
#include <XSControl_TransferReader.hxx>

#include <CommonCrypto/CommonDigest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <memory>
#include <mutex>
#include <istream>
#include <limits>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

NSErrorDomain const Core3DNativeImportErrorDomain =
    @"Core3DNativeImportErrorDomain";

namespace {

constexpr std::uint64_t kMaximumSourceBytes =
    32ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaximumArtifactBytes =
    96ULL * 1024ULL * 1024ULL;
constexpr Standard_Integer kMaximumSTEPEntities = 75'000;
constexpr std::uint64_t kMaximumSTEPHeaderRecords = 1'024;
constexpr std::uint64_t kMaximumSTEPParserNodes = 750'000;
constexpr std::uint64_t kMaximumSTEPParserNodesPerRecord = 8'192;
constexpr std::uint64_t kMaximumSTEPParenthesisDepth = 64;
constexpr std::uint64_t kMaximumSTEPTokenBytes = 256ULL * 1024ULL;
constexpr std::uint64_t kMaximumSTEPStringBytes = 256ULL * 1024ULL;
constexpr std::uint64_t kMaximumSTEPCommentBytes = 256ULL * 1024ULL;
constexpr std::uint64_t kMaximumSTEPStatementBytes = 512ULL * 1024ULL;
constexpr std::uint64_t kMaximumEstimatedSTEPParserBytes =
    64ULL * 1024ULL * 1024ULL;
constexpr Standard_Integer kMaximumRoots = 4'096;
constexpr Standard_Integer kMaximumOccurrenceDepth = 128;
constexpr std::size_t kMaximumLabelInstanceMappings = 1'000'000;
constexpr Standard_Size kMaximumAssemblyTraversalNodes = 32'768;
constexpr Standard_Size kMaximumShapeDefinitions = 4'096;
// Imported topology is validated explicitly below instead of entering OCCT's
// uninterruptible geometric BRep checker. These ceilings bound every walk.
constexpr Standard_Size kMaximumSubshapesPerDefinition = 8'192;
constexpr Standard_Size kMaximumAggregateSubshapes = 131'072;
constexpr Standard_Size kMaximumDocumentLabels = 100'000;
constexpr Standard_Integer kMaximumMaterialDefinitions = 2'048;

struct NativeImportState {
    std::string sourcePath;
    Core3DNativeImportFormat format = Core3DNativeImportFormatSTEP;
    std::string cleanupPath;
    std::string stagingDirectoryPath;
    std::string stagedSourcePath;
    std::string packageRootPath;
    std::string primaryPath;
    std::atomic_bool cancelled{false};
    std::mutex lifecycleMutex;
    bool started = false;
    bool finished = false;
    std::size_t shapeCount = 0;
};

struct NativeImportResult {
    bool succeeded = false;
    Core3DNativeImportErrorCode errorCode =
        Core3DNativeImportErrorInternalFailure;
    std::string message;
};

class NativeImportFailure final : public std::runtime_error {
public:
    NativeImportFailure(
        const Core3DNativeImportErrorCode code,
        const std::string& message)
    : std::runtime_error(message), myCode(code) {
    }

    Core3DNativeImportErrorCode Code() const noexcept {
        return myCode;
    }

private:
    Core3DNativeImportErrorCode myCode;
};

class NativeImportProgress final : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(
        NativeImportProgress,
        Message_ProgressIndicator)

public:
    explicit NativeImportProgress(const std::atomic_bool *cancelled)
    : myCancelled(cancelled) {
    }

protected:
    Standard_Boolean UserBreak() override {
        return myCancelled != nullptr
            && myCancelled->load(std::memory_order_acquire)
            ? Standard_True
            : Standard_False;
    }

    void Show(
        const Message_ProgressScope&,
        const Standard_Boolean) override {
    }

private:
    const std::atomic_bool *myCancelled;
};

NSOperationQueue *NativeImportQueue() {
    static NSOperationQueue *queue = nil;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
        queue = [[NSOperationQueue alloc] init];
        queue.name = @"app.shapeyard.core3d.native-import";
        queue.qualityOfService = NSQualityOfServiceUserInitiated;
        // Two independent documents may progress concurrently. The STEP
        // reader/writer lifecycle remains serialized by its narrower mutex.
        queue.maxConcurrentOperationCount = 2;
    });
    return queue;
}

dispatch_queue_t NativeImportCleanupQueue() {
    static dispatch_queue_t queue = dispatch_queue_create(
        "app.shapeyard.core3d.native-import-cleanup",
        dispatch_queue_attr_make_with_qos_class(
            DISPATCH_QUEUE_CONCURRENT,
            QOS_CLASS_UTILITY,
            0));
    return queue;
}

#ifdef DEBUG
std::mutex gDebugWorkerPauseMutex;
std::condition_variable gDebugWorkerPauseCondition;
bool gDebugWorkerPaused = false;
std::mutex gDebugSourceIdentityMutex;
std::condition_variable gDebugSourceIdentityCondition;
bool gDebugSourceIdentityPaused = false;
bool gDebugSourceIdentityWaiting = false;
std::mutex gDebugStructuralValidationMutex;
std::condition_variable gDebugStructuralValidationCondition;
bool gDebugStructuralValidationPaused = false;
bool gDebugStructuralValidationClaimed = false;
bool gDebugStructuralValidationWaiting = false;

void WaitForDebugWorkerBarrier(
    const std::shared_ptr<NativeImportState>& state) {
    std::unique_lock<std::mutex> lock(gDebugWorkerPauseMutex);
    gDebugWorkerPauseCondition.wait(lock, [&state] {
        return !gDebugWorkerPaused
            || state->cancelled.load(std::memory_order_acquire);
    });
}

void WaitForDebugSourceIdentityBarrier(
    const std::shared_ptr<NativeImportState>& state) {
    std::unique_lock<std::mutex> lock(gDebugSourceIdentityMutex);
    if (!gDebugSourceIdentityPaused) {
        return;
    }
    gDebugSourceIdentityWaiting = true;
    gDebugSourceIdentityCondition.notify_all();
    gDebugSourceIdentityCondition.wait(lock, [&state] {
        return !gDebugSourceIdentityPaused
            || state->cancelled.load(std::memory_order_acquire);
    });
    gDebugSourceIdentityWaiting = false;
    gDebugSourceIdentityCondition.notify_all();
}

void WaitForDebugStructuralValidationBarrier(
    const std::shared_ptr<NativeImportState>& state) {
    std::unique_lock<std::mutex> lock(gDebugStructuralValidationMutex);
    if (!gDebugStructuralValidationPaused
        || gDebugStructuralValidationClaimed) {
        return;
    }
    gDebugStructuralValidationClaimed = true;
    gDebugStructuralValidationWaiting = true;
    gDebugStructuralValidationCondition.notify_all();
    gDebugStructuralValidationCondition.wait(lock, [&state] {
        return !gDebugStructuralValidationPaused
            || state->cancelled.load(std::memory_order_acquire);
    });
    gDebugStructuralValidationWaiting = false;
    gDebugStructuralValidationCondition.notify_all();
}
#else
void WaitForDebugWorkerBarrier(
    const std::shared_ptr<NativeImportState>&) {
}

void WaitForDebugSourceIdentityBarrier(
    const std::shared_ptr<NativeImportState>&) {
}

void WaitForDebugStructuralValidationBarrier(
    const std::shared_ptr<NativeImportState>&) {
}
#endif

void RemoveTreeNoThrow(const std::string& path) noexcept {
    if (path.empty()) {
        return;
    }
    std::error_code error;
    std::filesystem::remove_all(std::filesystem::path(path), error);
}

void ThrowIfCancelled(
    const std::shared_ptr<NativeImportState>& state) {
    if (state->cancelled.load(std::memory_order_acquire)) {
        throw NativeImportFailure(
            Core3DNativeImportErrorCancelled,
            "The STEP import was cancelled.");
    }
}

bool IsSTEPPath(const std::filesystem::path& path) {
    std::string extension = path.extension().string();
    for (char& character : extension) {
        if (character >= 'A' && character <= 'Z') {
            character = static_cast<char>(character - 'A' + 'a');
        }
    }
    return extension == ".step" || extension == ".stp";
}

class FileDescriptor {
public:
    explicit FileDescriptor(const int descriptor = -1)
    : myDescriptor(descriptor) {
    }

    ~FileDescriptor() {
        if (myDescriptor >= 0) {
            ::close(myDescriptor);
        }
    }

    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;

    FileDescriptor(FileDescriptor&& other) noexcept
    : myDescriptor(std::exchange(other.myDescriptor, -1)) {
    }

    FileDescriptor& operator=(FileDescriptor&& other) noexcept {
        if (this != &other) {
            Close();
            myDescriptor = std::exchange(other.myDescriptor, -1);
        }
        return *this;
    }

    int Get() const { return myDescriptor; }
    bool IsValid() const { return myDescriptor >= 0; }

    void Close() noexcept {
        if (myDescriptor >= 0) {
            ::close(myDescriptor);
            myDescriptor = -1;
        }
    }

private:
    int myDescriptor;
};

struct FileIdentity {
    dev_t device = 0;
    ino_t inode = 0;
    off_t size = 0;
    mode_t mode = 0;
    nlink_t linkCount = 0;
    timespec modificationTime = {};
    timespec changeTime = {};
};

FileIdentity IdentityFromStat(const struct stat& info) noexcept {
    return {
        info.st_dev,
        info.st_ino,
        info.st_size,
        info.st_mode,
        info.st_nlink,
        info.st_mtimespec,
        info.st_ctimespec,
    };
}

bool SameTimespec(
    const timespec& left,
    const timespec& right) noexcept {
    return left.tv_sec == right.tv_sec && left.tv_nsec == right.tv_nsec;
}

bool SameFileIdentity(
    const FileIdentity& left,
    const FileIdentity& right) noexcept {
    return left.device == right.device
        && left.inode == right.inode
        && left.size == right.size
        && left.mode == right.mode
        && left.linkCount == right.linkCount
        && SameTimespec(left.modificationTime, right.modificationTime)
        && SameTimespec(left.changeTime, right.changeTime);
}

bool SameFileContentMetadata(
    const FileIdentity& left,
    const FileIdentity& right) noexcept {
    return left.device == right.device
        && left.inode == right.inode
        && left.size == right.size
        && left.mode == right.mode
        && SameTimespec(left.modificationTime, right.modificationTime);
}

using SHA256Digest =
    std::array<unsigned char, CC_SHA256_DIGEST_LENGTH>;

class SHA256Accumulator final {
public:
    SHA256Accumulator()
    : myValid(CC_SHA256_Init(&myContext) == 1) {
    }

    bool IsValid() const noexcept {
        return myValid;
    }

    bool Update(const void *bytes, const std::size_t count) noexcept {
        if (!myValid || myFinished || (bytes == nullptr && count != 0)) {
            return false;
        }
        const unsigned char *cursor =
            static_cast<const unsigned char *>(bytes);
        std::size_t remaining = count;
        while (remaining > 0) {
            const CC_LONG chunk = static_cast<CC_LONG>(std::min<std::size_t>(
                remaining,
                static_cast<std::size_t>(std::numeric_limits<CC_LONG>::max())));
            if (CC_SHA256_Update(&myContext, cursor, chunk) != 1) {
                myValid = false;
                return false;
            }
            cursor += chunk;
            remaining -= chunk;
        }
        return true;
    }

    bool Finish(SHA256Digest& digest) noexcept {
        if (!myValid || myFinished) {
            return false;
        }
        myFinished = true;
        if (CC_SHA256_Final(digest.data(), &myContext) != 1) {
            myValid = false;
            return false;
        }
        return true;
    }

private:
    CC_SHA256_CTX myContext = {};
    bool myValid = false;
    bool myFinished = false;
};

struct PinnedStagedFile {
    FileDescriptor descriptor;
    FileIdentity identity;
    SHA256Digest digest = {};

    PinnedStagedFile(
        FileDescriptor&& fileDescriptor,
        const FileIdentity& fileIdentity,
        const SHA256Digest& fileDigest)
    : descriptor(std::move(fileDescriptor)),
      identity(fileIdentity),
      digest(fileDigest) {
    }

    PinnedStagedFile(PinnedStagedFile&&) noexcept = default;
    PinnedStagedFile& operator=(PinnedStagedFile&&) noexcept = default;
    PinnedStagedFile(const PinnedStagedFile&) = delete;
    PinnedStagedFile& operator=(const PinnedStagedFile&) = delete;
};

class PrivateDocumentGuard {
public:
    explicit PrivateDocumentGuard(const Handle(OcctDocument)& document)
    : myDocument(document) {
    }

    ~PrivateDocumentGuard() {
        Close();
    }

    PrivateDocumentGuard(const PrivateDocumentGuard&) = delete;
    PrivateDocumentGuard& operator=(const PrivateDocumentGuard&) = delete;

    void Close() noexcept {
        if (!myDocument.IsNull()) {
            myDocument->ClosePrivateExportSnapshot();
            myDocument.Nullify();
        }
    }

private:
    Handle(OcctDocument) myDocument;
};

class CancellableFileStreamBuffer final : public std::streambuf {
public:
    CancellableFileStreamBuffer(
        const int descriptor,
        const std::shared_ptr<NativeImportState>& state,
        const FileIdentity& expectedIdentity,
        const SHA256Digest& expectedDigest)
    : myDescriptor(descriptor),
      myState(state),
      myExpectedIdentity(expectedIdentity),
      myExpectedDigest(expectedDigest) {
        setg(myBuffer.data(), myBuffer.data(), myBuffer.data());
        struct stat info = {};
        myIdentityValid = myDescriptor >= 0
            && ::fstat(myDescriptor, &info) == 0
            && S_ISREG(info.st_mode)
            && SameFileIdentity(
                IdentityFromStat(info), myExpectedIdentity)
            && myDigest.IsValid();
    }

    bool IsOpen() const noexcept {
        return myDescriptor >= 0 && myIdentityValid;
    }

    bool ReadFailed() const noexcept {
        return myReadFailed;
    }

    bool FinishValidation() noexcept {
        if (myFinalized) {
            return myValidationSucceeded;
        }
        // Bytes already buffered for the parser were hashed at read time. The
        // parser may stop at END-ISO-10303-21, so drain only the unread file
        // descriptor tail before comparing the full staged-file digest.
        setg(myBuffer.data(), myBuffer.data(), myBuffer.data());
        while (true) {
            if (myState->cancelled.load(std::memory_order_acquire)) {
                return false;
            }
            ssize_t readCount = 0;
            do {
                readCount = ::read(
                    myDescriptor, myBuffer.data(), myBuffer.size());
            } while (readCount < 0 && errno == EINTR);
            if (readCount < 0) {
                myReadFailed = true;
                return Finalize(false);
            }
            if (readCount == 0) {
                return Finalize(true);
            }
            if (!RecordRead(readCount)) {
                myReadFailed = true;
                return Finalize(false);
            }
        }
    }

protected:
    int_type underflow() override {
        if (gptr() != nullptr && gptr() < egptr()) {
            return traits_type::to_int_type(*gptr());
        }
        if (myState->cancelled.load(std::memory_order_acquire)) {
            return traits_type::eof();
        }
        ssize_t readCount = 0;
        do {
            readCount = ::read(
                myDescriptor, myBuffer.data(), myBuffer.size());
        } while (readCount < 0 && errno == EINTR);
        if (readCount < 0) {
            myReadFailed = true;
            return traits_type::eof();
        }
        if (myState->cancelled.load(std::memory_order_acquire)) {
            return traits_type::eof();
        }
        if (readCount == 0) {
            if (!Finalize(true)) {
                myReadFailed = true;
            }
            return traits_type::eof();
        }
        if (!RecordRead(readCount)) {
            myReadFailed = true;
            return traits_type::eof();
        }
        setg(
            myBuffer.data(),
            myBuffer.data(),
            myBuffer.data() + readCount);
        return traits_type::to_int_type(*gptr());
    }

private:
    bool RecordRead(const ssize_t readCount) noexcept {
        if (readCount <= 0
            || myExpectedIdentity.size <= 0
            || myReadBytes
                > static_cast<std::uint64_t>(myExpectedIdentity.size)
            || static_cast<std::uint64_t>(readCount)
                > static_cast<std::uint64_t>(myExpectedIdentity.size)
                    - myReadBytes
            || !myDigest.Update(
                myBuffer.data(), static_cast<std::size_t>(readCount))) {
            return false;
        }
        myReadBytes += static_cast<std::uint64_t>(readCount);
        return true;
    }

    bool Finalize(const bool reachedEnd) noexcept {
        if (myFinalized) {
            return myValidationSucceeded;
        }
        myFinalized = true;
        struct stat info = {};
        SHA256Digest actualDigest = {};
        myValidationSucceeded = reachedEnd
            && myIdentityValid
            && myExpectedIdentity.size > 0
            && myReadBytes
                == static_cast<std::uint64_t>(myExpectedIdentity.size)
            && ::fstat(myDescriptor, &info) == 0
            && S_ISREG(info.st_mode)
            && SameFileIdentity(
                IdentityFromStat(info), myExpectedIdentity)
            && myDigest.Finish(actualDigest)
            && actualDigest == myExpectedDigest;
        return myValidationSucceeded;
    }

    int myDescriptor = -1;
    std::shared_ptr<NativeImportState> myState;
    FileIdentity myExpectedIdentity;
    SHA256Digest myExpectedDigest = {};
    SHA256Accumulator myDigest;
    std::array<char, 4 * 1024> myBuffer = {};
    std::uint64_t myReadBytes = 0;
    bool myIdentityValid = false;
    bool myReadFailed = false;
    bool myFinalized = false;
    bool myValidationSucceeded = false;
};

enum class STEPPreflightResult {
    InProgress,
    Valid,
    Invalid,
    ResourceLimit,
    UnsupportedEncoding,
};

//! A deliberately small streaming lexer for the common single-DATA Part 21
//! encoding accepted by the bundled OCCT reader. Its primary job is to bound
//! every allocation-driving dimension before OCCT constructs StepReaderData;
//! it is not a replacement for OCCT's semantic parser.
class STEPTextPreflight final {
public:
    void Consume(const unsigned char *bytes, const std::size_t count) {
        for (std::size_t index = 0;
             index < count && myResult == STEPPreflightResult::InProgress;
             ++index) {
            if (!ChargeEstimatedBytes(1)) {
                break;
            }
            if (myStage == Stage::Complete) {
                // OCCT's lexer intentionally ignores Edition 3 signature data
                // after END-ISO-10303-21. Physical bytes remain budgeted.
                continue;
            }
            if (myStatementBytes >= kMaximumSTEPStatementBytes) {
                Fail(STEPPreflightResult::ResourceLimit);
                break;
            }
            ++myStatementBytes;
            ConsumeByte(bytes[index]);
        }
    }

    STEPPreflightResult Finish() {
        if (myResult != STEPPreflightResult::InProgress) {
            return myResult;
        }
        if (myStage == Stage::Complete) {
            myResult = myEntityCount > 0
                ? STEPPreflightResult::Valid
                : STEPPreflightResult::Invalid;
            return myResult;
        }
        if (myInComment) {
            return STEPPreflightResult::Invalid;
        }
        if (myInString) {
            if (!myStringQuotePending) {
                return STEPPreflightResult::Invalid;
            }
            myInString = false;
            myStringQuotePending = false;
        }
        if (myPendingSlash) {
            myPendingSlash = false;
            ProcessSymbol('/');
        }
        FinishToken();
        if (myResult != STEPPreflightResult::InProgress) {
            return myResult;
        }
        myResult = myStage == Stage::Complete
                && myEntityCount > 0
                && !myStatementHasMeaningfulContent
                && myParenthesisDepth == 0
            ? STEPPreflightResult::Valid
            : STEPPreflightResult::Invalid;
        return myResult;
    }

    STEPPreflightResult Result() const noexcept {
        return myResult;
    }

private:
    enum class Stage {
        Opening,
        HeaderMarker,
        HeaderRecords,
        DataMarker,
        DataRecords,
        Terminator,
        Complete,
    };

    static bool IsTokenCharacter(const unsigned char character) {
        return std::isalnum(character) != 0
            || character == '-'
            || character == '_'
            || character == '#';
    }

    static bool IsEntityIdentifier(const std::string& token) {
        if (token.size() < 2 || token.front() != '#') {
            return false;
        }
        for (std::size_t index = 1; index < token.size(); ++index) {
            if (token[index] < '0' || token[index] > '9') {
                return false;
            }
        }
        return true;
    }

    bool ChargeEstimatedBytes(const std::uint64_t delta) {
        if (delta > kMaximumEstimatedSTEPParserBytes
                - myEstimatedParserBytes) {
            Fail(STEPPreflightResult::ResourceLimit);
            return false;
        }
        myEstimatedParserBytes += delta;
        return true;
    }

    bool ChargeParserNode() {
        if (myParserNodeCount >= kMaximumSTEPParserNodes
            || myStatementParserNodeCount
                >= kMaximumSTEPParserNodesPerRecord) {
            Fail(STEPPreflightResult::ResourceLimit);
            return false;
        }
        ++myParserNodeCount;
        ++myStatementParserNodeCount;
        return ChargeEstimatedBytes(24);
    }

    void ConsumeByte(const unsigned char character) {
        if (myInComment) {
            if (myCommentBytes >= kMaximumSTEPCommentBytes) {
                Fail(STEPPreflightResult::ResourceLimit);
                return;
            }
            ++myCommentBytes;
            if (myCommentStar && character == '/') {
                myInComment = false;
                myCommentStar = false;
                myCommentBytes = 0;
            } else {
                myCommentStar = character == '*';
            }
            return;
        }
        if (myInString) {
            if (myStringQuotePending) {
                if (character == '\'') {
                    if (myStringBytes >= kMaximumSTEPStringBytes) {
                        Fail(STEPPreflightResult::ResourceLimit);
                        return;
                    }
                    ++myStringBytes;
                    myStringQuotePending = false;
                    return;
                }
                myInString = false;
                myStringQuotePending = false;
                myStringBytes = 0;
                ConsumeOutsideString(character);
                return;
            }
            if (myStringBytes >= kMaximumSTEPStringBytes) {
                Fail(STEPPreflightResult::ResourceLimit);
                return;
            }
            ++myStringBytes;
            if (character == '\'') {
                myStringQuotePending = true;
            }
            return;
        }
        ConsumeOutsideString(character);
    }

    void ConsumeOutsideString(const unsigned char character) {
        if (myPendingSlash) {
            myPendingSlash = false;
            if (character == '*') {
                FinishToken();
                myInComment = true;
                myCommentStar = false;
                myCommentBytes = 2;
                return;
            }
            ProcessSymbol('/');
            if (myResult != STEPPreflightResult::InProgress) {
                return;
            }
        }
        // OCCT's Part 21 lexer ignores NUL octets. Treat them as token
        // separators while retaining their physical/allocation charge.
        if (character == 0 || std::isspace(character) != 0) {
            FinishToken();
            return;
        }
        if (character == '/') {
            FinishToken();
            myPendingSlash = true;
            return;
        }
        if (character == '\'') {
            FinishToken();
            myStatementHasMeaningfulContent = true;
            myStatementHasSyntax = true;
            myInString = true;
            myStringBytes = 1;
            myStringQuotePending = false;
            ChargeParserNodeIfRecord();
            return;
        }
        if (IsTokenCharacter(character)) {
            ConsumeTokenCharacter(character);
            return;
        }
        ProcessSymbol(character);
    }

    void ConsumeTokenCharacter(const unsigned char character) {
        if (!myInToken) {
            myInToken = true;
            myTokenBytes = 0;
            myToken.clear();
            myStatementHasMeaningfulContent = true;
            ChargeParserNodeIfRecord();
        }
        if (myTokenBytes >= kMaximumSTEPTokenBytes) {
            Fail(STEPPreflightResult::ResourceLimit);
            return;
        }
        ++myTokenBytes;
        if (myToken.size() < 128) {
            myToken.push_back(static_cast<char>(std::toupper(character)));
        } else {
            myTokenOverflow = true;
        }
    }

    void FinishToken() {
        if (!myInToken) {
            return;
        }
        if (!myFirstTokenSet) {
            myFirstToken = myToken;
            myFirstTokenSet = true;
            myFirstTokenOverflow = myTokenOverflow;
        } else {
            myHasExtraTokens = true;
        }
        myInToken = false;
        myTokenBytes = 0;
        myToken.clear();
        myTokenOverflow = false;
    }

    void ProcessSymbol(const unsigned char character) {
        FinishToken();
        if (myResult != STEPPreflightResult::InProgress) {
            return;
        }
        if (character == ';') {
            if (myParenthesisDepth != 0) {
                Fail(STEPPreflightResult::Invalid);
                return;
            }
            FinalizeStatement();
            return;
        }
        myStatementHasMeaningfulContent = true;
        myStatementHasSyntax = true;
        switch (character) {
            case '(':
                if (myParenthesisDepth >= kMaximumSTEPParenthesisDepth) {
                    Fail(STEPPreflightResult::ResourceLimit);
                    return;
                }
                ++myParenthesisDepth;
                ChargeParserNodeIfRecord();
                break;
            case ')':
                if (myParenthesisDepth == 0) {
                    Fail(STEPPreflightResult::Invalid);
                    return;
                }
                --myParenthesisDepth;
                break;
            case ',':
                ChargeParserNodeIfRecord();
                break;
            case '=':
                mySawEquals = true;
                break;
            default:
                ChargeParserNodeIfRecord();
                break;
        }
    }

    void ChargeParserNodeIfRecord() {
        if (myStage == Stage::HeaderRecords
            || myStage == Stage::DataRecords) {
            ChargeParserNode();
        }
    }

    bool IsExactStatement(const char *token) const {
        return myFirstTokenSet
            && !myFirstTokenOverflow
            && myFirstToken == token
            && !myHasExtraTokens
            && !myStatementHasSyntax;
    }

    void FinalizeStatement() {
        FinishToken();
        if (!myStatementHasMeaningfulContent || myFirstTokenOverflow) {
            Fail(STEPPreflightResult::Invalid);
            return;
        }
        switch (myStage) {
            case Stage::Opening:
                if (!IsExactStatement("ISO-10303-21")) {
                    Fail(STEPPreflightResult::Invalid);
                    return;
                }
                myStage = Stage::HeaderMarker;
                break;
            case Stage::HeaderMarker:
                if (!IsExactStatement("HEADER")) {
                    Fail(STEPPreflightResult::Invalid);
                    return;
                }
                myStage = Stage::HeaderRecords;
                break;
            case Stage::HeaderRecords:
                if (IsExactStatement("ENDSEC")) {
                    myStage = Stage::DataMarker;
                    break;
                }
                if (!myFirstTokenSet) {
                    Fail(STEPPreflightResult::Invalid);
                    return;
                }
                if (myHeaderRecordCount >= kMaximumSTEPHeaderRecords
                    || !ChargeEstimatedBytes(256)) {
                    Fail(STEPPreflightResult::ResourceLimit);
                    return;
                }
                ++myHeaderRecordCount;
                break;
            case Stage::DataMarker:
                if (IsExactStatement("DATA")) {
                    myStage = Stage::DataRecords;
                    break;
                }
                if (myFirstToken == "DATA"
                    || myFirstToken == "ANCHOR"
                    || myFirstToken == "REFERENCE") {
                    Fail(STEPPreflightResult::UnsupportedEncoding);
                } else {
                    Fail(STEPPreflightResult::Invalid);
                }
                return;
            case Stage::DataRecords:
                if (IsExactStatement("ENDSEC")) {
                    myStage = Stage::Terminator;
                    break;
                }
                if (!IsEntityIdentifier(myFirstToken) || !mySawEquals) {
                    Fail(STEPPreflightResult::Invalid);
                    return;
                }
                if (myEntityCount
                        >= static_cast<std::uint64_t>(kMaximumSTEPEntities)
                    || !ChargeEstimatedBytes(256)) {
                    Fail(STEPPreflightResult::ResourceLimit);
                    return;
                }
                ++myEntityCount;
                break;
            case Stage::Terminator:
                if (IsExactStatement("END-ISO-10303-21")) {
                    myStage = Stage::Complete;
                    break;
                }
                if (myFirstToken == "DATA") {
                    Fail(STEPPreflightResult::UnsupportedEncoding);
                } else {
                    Fail(STEPPreflightResult::Invalid);
                }
                return;
            case Stage::Complete:
                break;
        }
        ResetStatement();
    }

    void ResetStatement() {
        myStatementBytes = 0;
        myStatementParserNodeCount = 0;
        myStatementHasMeaningfulContent = false;
        myStatementHasSyntax = false;
        myFirstTokenSet = false;
        myFirstTokenOverflow = false;
        myFirstToken.clear();
        myHasExtraTokens = false;
        mySawEquals = false;
        myParenthesisDepth = 0;
        myInToken = false;
        myTokenBytes = 0;
        myToken.clear();
        myTokenOverflow = false;
    }

    void Fail(const STEPPreflightResult result) {
        if (myResult == STEPPreflightResult::InProgress) {
            myResult = result;
        }
    }

    Stage myStage = Stage::Opening;
    STEPPreflightResult myResult = STEPPreflightResult::InProgress;
    std::uint64_t myEntityCount = 0;
    std::uint64_t myHeaderRecordCount = 0;
    std::uint64_t myParserNodeCount = 0;
    std::uint64_t myEstimatedParserBytes = 0;
    std::uint64_t myStatementBytes = 0;
    std::uint64_t myStatementParserNodeCount = 0;
    std::uint64_t myParenthesisDepth = 0;
    std::uint64_t myTokenBytes = 0;
    std::uint64_t myStringBytes = 0;
    std::uint64_t myCommentBytes = 0;
    std::string myToken;
    std::string myFirstToken;
    bool myInToken = false;
    bool myTokenOverflow = false;
    bool myFirstTokenSet = false;
    bool myFirstTokenOverflow = false;
    bool myHasExtraTokens = false;
    bool myStatementHasMeaningfulContent = false;
    bool myStatementHasSyntax = false;
    bool mySawEquals = false;
    bool myPendingSlash = false;
    bool myInComment = false;
    bool myCommentStar = false;
    bool myInString = false;
    bool myStringQuotePending = false;
};

PinnedStagedFile CopySourceIntoPrivateStaging(
    const std::shared_ptr<NativeImportState>& state) {
    ThrowIfCancelled(state);
    if (!IsSTEPPath(std::filesystem::path(state->sourcePath))) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "Choose a STEP or STP file.");
    }

    struct stat pathInfo = {};
    if (::lstat(state->sourcePath.c_str(), &pathInfo) != 0
        || !S_ISREG(pathInfo.st_mode)
        || pathInfo.st_size <= 0) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "The selected STEP file is empty or unsafe.");
    }
    if (static_cast<std::uint64_t>(pathInfo.st_size)
        > kMaximumSourceBytes) {
        throw NativeImportFailure(
            Core3DNativeImportErrorResourceLimit,
            "The selected STEP file exceeds the 32 MiB mobile import limit.");
    }

    FileDescriptor source(::open(
        state->sourcePath.c_str(),
        O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
    if (!source.IsValid()) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "The selected STEP file could not be opened safely.");
    }
    struct stat openedInfo = {};
    if (::fstat(source.Get(), &openedInfo) != 0
        || !S_ISREG(openedInfo.st_mode)
        || !SameFileIdentity(
            IdentityFromStat(openedInfo), IdentityFromStat(pathInfo))) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "The selected STEP file changed while it was being opened.");
    }

    FileDescriptor pinnedReader;
    FileIdentity stagedIdentityBeforeUnlink;
    SHA256Digest stagedDigest = {};
    {
        FileDescriptor destination(::open(
            state->stagedSourcePath.c_str(),
            O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
            S_IRUSR | S_IWUSR));
        if (!destination.IsValid()) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInternalFailure,
                "Private STEP staging could not be created.");
        }
        SHA256Accumulator copyDigest;
        if (!copyDigest.IsValid()) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInternalFailure,
                "Private STEP staging could not initialize validation.");
        }

        std::array<unsigned char, 64 * 1024> buffer = {};
        std::uint64_t copiedBytes = 0;
        bool reachedIdentityBarrier = false;
        while (true) {
            ThrowIfCancelled(state);
            ssize_t readCount = ::read(
                source.Get(), buffer.data(), buffer.size());
            if (readCount < 0 && errno == EINTR) {
                continue;
            }
            if (readCount < 0) {
                throw NativeImportFailure(
                    Core3DNativeImportErrorInvalidSource,
                    "The selected STEP file could not be read.");
            }
            if (readCount == 0) {
                break;
            }
            copiedBytes += static_cast<std::uint64_t>(readCount);
            if (copiedBytes > static_cast<std::uint64_t>(openedInfo.st_size)
                || copiedBytes > kMaximumSourceBytes
                || !copyDigest.Update(
                    buffer.data(), static_cast<std::size_t>(readCount))) {
                throw NativeImportFailure(
                    Core3DNativeImportErrorInvalidSource,
                    "The selected STEP file changed during staging.");
            }
            ssize_t writtenBytes = 0;
            while (writtenBytes < readCount) {
                ThrowIfCancelled(state);
                const ssize_t writeCount = ::write(
                    destination.Get(),
                    buffer.data() + writtenBytes,
                    static_cast<std::size_t>(readCount - writtenBytes));
                if (writeCount < 0 && errno == EINTR) {
                    continue;
                }
                if (writeCount <= 0) {
                    throw NativeImportFailure(
                        Core3DNativeImportErrorInternalFailure,
                        "The STEP staging copy could not be written.");
                }
                writtenBytes += writeCount;
            }
            if (!reachedIdentityBarrier) {
                reachedIdentityBarrier = true;
                WaitForDebugSourceIdentityBarrier(state);
                ThrowIfCancelled(state);
            }
        }

        struct stat finalSourceInfo = {};
        int syncResult = 0;
        do {
            syncResult = ::fsync(destination.Get());
        } while (syncResult != 0 && errno == EINTR);
        if (copiedBytes != static_cast<std::uint64_t>(openedInfo.st_size)
            || ::fstat(source.Get(), &finalSourceInfo) != 0
            || !SameFileIdentity(
                IdentityFromStat(finalSourceInfo),
                IdentityFromStat(openedInfo))
            || syncResult != 0
            || !copyDigest.Finish(stagedDigest)) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInvalidSource,
                "The selected STEP file changed during staging.");
        }

        struct stat stagedWriterInfo = {};
        if (::fstat(destination.Get(), &stagedWriterInfo) != 0
            || !S_ISREG(stagedWriterInfo.st_mode)
            || stagedWriterInfo.st_size != openedInfo.st_size
            || stagedWriterInfo.st_nlink != 1) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInternalFailure,
                "The private STEP staging file is unsafe.");
        }
        stagedIdentityBeforeUnlink = IdentityFromStat(stagedWriterInfo);

        FileDescriptor reader(::open(
            state->stagedSourcePath.c_str(),
            O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
        struct stat stagedReaderInfo = {};
        if (!reader.IsValid()
            || ::fstat(reader.Get(), &stagedReaderInfo) != 0
            || !S_ISREG(stagedReaderInfo.st_mode)
            || !SameFileIdentity(
                IdentityFromStat(stagedReaderInfo),
                stagedIdentityBeforeUnlink)) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInternalFailure,
                "The private STEP staging file could not be pinned safely.");
        }
        pinnedReader = std::move(reader);
    }

    // Remove the only directory entry before lexical validation. OCCT receives
    // this same read-only descriptor, so no mutable path exists between the
    // preflight and semantic parser passes.
    if (::unlink(state->stagedSourcePath.c_str()) != 0) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInternalFailure,
            "The private STEP staging file could not be isolated.");
    }
    struct stat pinnedInfo = {};
    if (::fstat(pinnedReader.Get(), &pinnedInfo) != 0
        || !S_ISREG(pinnedInfo.st_mode)
        || pinnedInfo.st_nlink != 0
        || !SameFileContentMetadata(
            IdentityFromStat(pinnedInfo), stagedIdentityBeforeUnlink)) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "The private STEP staging file changed while it was isolated.");
    }
    RemoveTreeNoThrow(state->stagingDirectoryPath);
    return PinnedStagedFile(
        std::move(pinnedReader), IdentityFromStat(pinnedInfo), stagedDigest);
}

void PreflightStagedSTEP(
    const std::shared_ptr<NativeImportState>& state,
    const PinnedStagedFile& stagedFile) {
    ThrowIfCancelled(state);
    if (!stagedFile.descriptor.IsValid()
        || ::lseek(stagedFile.descriptor.Get(), 0, SEEK_SET) != 0) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInternalFailure,
            "The private STEP staging file could not be rewound.");
    }
    struct stat initialInfo = {};
    if (::fstat(stagedFile.descriptor.Get(), &initialInfo) != 0
        || !S_ISREG(initialInfo.st_mode)
        || initialInfo.st_size <= 0
        || static_cast<std::uint64_t>(initialInfo.st_size)
            > kMaximumSourceBytes
        || !SameFileIdentity(
            IdentityFromStat(initialInfo), stagedFile.identity)) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "The private STEP staging file changed before validation.");
    }

    STEPTextPreflight preflight;
    SHA256Accumulator preflightDigest;
    if (!preflightDigest.IsValid()) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInternalFailure,
            "The staged STEP validator could not initialize.");
    }
    std::array<unsigned char, 64 * 1024> buffer = {};
    std::uint64_t readBytes = 0;
    while (true) {
        ThrowIfCancelled(state);
        ssize_t readCount = ::read(
            stagedFile.descriptor.Get(), buffer.data(), buffer.size());
        if (readCount < 0 && errno == EINTR) {
            continue;
        }
        if (readCount < 0) {
            throw NativeImportFailure(
                Core3DNativeImportErrorReadFailed,
                "The staged STEP file could not be checked safely.");
        }
        if (readCount == 0) {
            break;
        }
        if (static_cast<std::uint64_t>(readCount)
                > static_cast<std::uint64_t>(initialInfo.st_size) - readBytes) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInvalidSource,
                "The private STEP staging file changed during validation.");
        }
        readBytes += static_cast<std::uint64_t>(readCount);
        if (!preflightDigest.Update(
                buffer.data(), static_cast<std::size_t>(readCount))) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInternalFailure,
                "The staged STEP digest could not be calculated.");
        }
        preflight.Consume(
            buffer.data(), static_cast<std::size_t>(readCount));
        switch (preflight.Result()) {
            case STEPPreflightResult::ResourceLimit:
                throw NativeImportFailure(
                    Core3DNativeImportErrorResourceLimit,
                    "The STEP model exceeds bounded mobile parser limits.");
            case STEPPreflightResult::UnsupportedEncoding:
                throw NativeImportFailure(
                    Core3DNativeImportErrorUnsupportedEncoding,
                    "This STEP encoding is not supported yet; choose a self-contained file with one unnamed DATA section.");
            case STEPPreflightResult::Invalid:
                throw NativeImportFailure(
                    Core3DNativeImportErrorReadFailed,
                    "The selected file is not a structurally valid STEP Part 21 model.");
            case STEPPreflightResult::InProgress:
            case STEPPreflightResult::Valid:
                break;
        }
    }
    ThrowIfCancelled(state);
    struct stat finalInfo = {};
    SHA256Digest actualDigest = {};
    if (readBytes != static_cast<std::uint64_t>(initialInfo.st_size)
        || ::fstat(stagedFile.descriptor.Get(), &finalInfo) != 0
        || !SameFileIdentity(
            IdentityFromStat(finalInfo), stagedFile.identity)
        || !preflightDigest.Finish(actualDigest)
        || actualDigest != stagedFile.digest) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidSource,
            "The private STEP staging file changed during validation.");
    }
    switch (preflight.Finish()) {
        case STEPPreflightResult::Valid:
            if (::lseek(stagedFile.descriptor.Get(), 0, SEEK_SET) != 0) {
                throw NativeImportFailure(
                    Core3DNativeImportErrorInternalFailure,
                    "The private STEP staging file could not be rewound.");
            }
            return;
        case STEPPreflightResult::ResourceLimit:
            throw NativeImportFailure(
                Core3DNativeImportErrorResourceLimit,
                "The STEP model exceeds bounded mobile parser limits.");
        case STEPPreflightResult::UnsupportedEncoding:
            throw NativeImportFailure(
                Core3DNativeImportErrorUnsupportedEncoding,
                "This STEP encoding is not supported yet; choose a self-contained file with one unnamed DATA section.");
        case STEPPreflightResult::InProgress:
        case STEPPreflightResult::Invalid:
            throw NativeImportFailure(
                Core3DNativeImportErrorReadFailed,
                "The selected file is not a complete STEP Part 21 model.");
    }
}

enum class ImportedDocumentValidation {
    Valid,
    Invalid,
    NoGeometry,
    ResourceLimit,
    MissingIdentifiers,
};

enum class BoundedProjectValidation {
    Valid,
    Invalid,
    ResourceLimit,
};

bool IsValidTopologyOrientation(
    const TopAbs_Orientation orientation) noexcept {
    switch (orientation) {
        case TopAbs_FORWARD:
        case TopAbs_REVERSED:
        case TopAbs_INTERNAL:
        case TopAbs_EXTERNAL:
            return true;
    }
    return false;
}

bool IsAllowedTopologyChild(
    const TopAbs_ShapeEnum parent,
    const TopAbs_ShapeEnum child) noexcept {
    switch (parent) {
        case TopAbs_COMPOUND:
            return child >= TopAbs_COMPOUND && child < TopAbs_SHAPE;
        case TopAbs_COMPSOLID:
            return child == TopAbs_SOLID;
        case TopAbs_SOLID:
            return child == TopAbs_SHELL;
        case TopAbs_SHELL:
            return child == TopAbs_FACE;
        case TopAbs_FACE:
            return child == TopAbs_WIRE;
        case TopAbs_WIRE:
            return child == TopAbs_EDGE;
        case TopAbs_EDGE:
            return child == TopAbs_VERTEX;
        case TopAbs_VERTEX:
        case TopAbs_SHAPE:
            return false;
    }
    return false;
}

BoundedProjectValidation ValidateBoundedTopologicalShape(
    const TopoDS_Shape& shape,
    const std::shared_ptr<NativeImportState>& state,
    Standard_Size& aggregateSubshapes) {
    if (shape.IsNull()) {
        return BoundedProjectValidation::Invalid;
    }
    try {
        OCC_CATCH_SIGNALS
        struct TopologyFrame {
            TopoDS_Shape shape;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        std::vector<TopologyFrame> stack = {{shape, 0, false}};
        TopTools_MapOfShape visited;
        TopTools_MapOfShape activePath;
        Standard_Size count = 0;
        while (!stack.empty()) {
            const TopologyFrame frame = stack.back();
            stack.pop_back();
            if (frame.shape.IsNull()) {
                return BoundedProjectValidation::Invalid;
            }
            if (frame.leaving) {
                activePath.Remove(frame.shape);
                visited.Add(frame.shape);
                continue;
            }
            if (visited.Contains(frame.shape)) {
                continue;
            }
            if (activePath.Contains(frame.shape)
                || frame.shape.ShapeType() == TopAbs_SHAPE
                || !IsValidTopologyOrientation(frame.shape.Orientation())) {
                return BoundedProjectValidation::Invalid;
            }
            if (frame.depth
                > static_cast<Standard_Size>(kMaximumOccurrenceDepth)) {
                return BoundedProjectValidation::ResourceLimit;
            }
            if (count >= kMaximumSubshapesPerDefinition
                || aggregateSubshapes >= kMaximumAggregateSubshapes) {
                return BoundedProjectValidation::ResourceLimit;
            }
            activePath.Add(frame.shape);
            ++count;
            if (count == 1U) {
                // This one-shot test hook is deliberately inside the actual
                // structural walk and after the STEP exchange lock is gone.
                WaitForDebugStructuralValidationBarrier(state);
            }
            if ((count & 63U) == 0U || count == 1U) {
                ThrowIfCancelled(state);
            }
            if (count > kMaximumAggregateSubshapes - aggregateSubshapes) {
                return BoundedProjectValidation::ResourceLimit;
            }
            if (stack.size()
                >= static_cast<std::size_t>(
                    kMaximumSubshapesPerDefinition) * 2U) {
                return BoundedProjectValidation::ResourceLimit;
            }
            stack.push_back({frame.shape, frame.depth, true});
            const TopAbs_ShapeEnum parentType = frame.shape.ShapeType();
            for (TopoDS_Iterator child(
                     frame.shape, Standard_True, Standard_True);
                 child.More(); child.Next()) {
                const TopoDS_Shape& childShape = child.Value();
                if (childShape.IsNull()
                    || !IsAllowedTopologyChild(
                        parentType, childShape.ShapeType())
                    || !IsValidTopologyOrientation(
                        childShape.Orientation())) {
                    return BoundedProjectValidation::Invalid;
                }
                if (stack.size()
                    >= static_cast<std::size_t>(
                        kMaximumSubshapesPerDefinition) * 2U) {
                    return BoundedProjectValidation::ResourceLimit;
                }
                stack.push_back({childShape, frame.depth + 1, false});
            }
        }
        aggregateSubshapes += count;
        ThrowIfCancelled(state);
        // Do not call BRepCheck_Analyzer here. Its geometric checks evaluate
        // attacker-controlled curves and surfaces without a progress hook.
        // STEP transfer diagnostics plus this bounded canonical topology walk
        // form the cancellable structural gate for the editable document.
        return BoundedProjectValidation::Valid;
    } catch (const NativeImportFailure&) {
        throw;
    } catch (const std::bad_alloc&) {
        throw;
    } catch (...) {
        return BoundedProjectValidation::Invalid;
    }
}

struct ShapeTraversalFrame {
    TDF_Label label;
    Standard_Size depth = 0;
    bool leaving = false;
};

BoundedProjectValidation ValidateEditorCompatibleShapeTree(
    const Handle(TDocStd_Document)& document,
    const std::shared_ptr<NativeImportState>& state) {
    if (document.IsNull()) {
        return BoundedProjectValidation::Invalid;
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (shapeTool.IsNull()) {
        return BoundedProjectValidation::Invalid;
    }

    Standard_Size documentLabelCount = 0;
    for (TDF_ChildIterator label(document->Main(), Standard_True);
         label.More(); label.Next()) {
        if (++documentLabelCount > kMaximumDocumentLabels) {
            return BoundedProjectValidation::ResourceLimit;
        }
        if ((documentLabelCount & 4095U) == 0U) {
            ThrowIfCancelled(state);
        }
    }

    std::vector<ShapeTraversalFrame> stack;
    stack.reserve(64);
    for (TDF_ChildIterator root(shapeTool->Label(), Standard_False);
         root.More(); root.Next()) {
        const TDF_Label& label = root.Value();
        if (!label.IsNull() && XCAFDoc_ShapeTool::IsShape(label)) {
            if (stack.size()
                >= static_cast<std::size_t>(kMaximumShapeDefinitions)) {
                return BoundedProjectValidation::ResourceLimit;
            }
            stack.push_back({label, 0, false});
        }
    }

    TDF_LabelMap activePath;
    TDF_LabelMap visitedLabels;
    Standard_Size traversalNodes = 0;
    Standard_Size definitionCount = 0;
    Standard_Size aggregateSubshapes = 0;
    while (!stack.empty()) {
        const ShapeTraversalFrame frame = stack.back();
        stack.pop_back();
        if (frame.leaving) {
            activePath.Remove(frame.label);
            visitedLabels.Add(frame.label);
            continue;
        }
        if (visitedLabels.Contains(frame.label)) {
            continue;
        }
        if (frame.label.IsNull() || activePath.Contains(frame.label)) {
            return BoundedProjectValidation::Invalid;
        }
        if (frame.depth > static_cast<Standard_Size>(kMaximumOccurrenceDepth)
            || ++traversalNodes > kMaximumAssemblyTraversalNodes) {
            return BoundedProjectValidation::ResourceLimit;
        }
        if ((traversalNodes & 1023U) == 0U) {
            ThrowIfCancelled(state);
        }

        activePath.Add(frame.label);
        stack.push_back({frame.label, frame.depth, true});
        if (XCAFDoc_ShapeTool::IsReference(frame.label)
            || XCAFDoc_ShapeTool::IsComponent(frame.label)) {
            TDF_Label referredLabel;
            if (!XCAFDoc_ShapeTool::GetReferredShape(
                    frame.label, referredLabel)
                || referredLabel.IsNull()
                || referredLabel.Data() != frame.label.Data()
                || !XCAFDoc_ShapeTool::IsShape(referredLabel)) {
                return BoundedProjectValidation::Invalid;
            }
            stack.push_back({referredLabel, frame.depth + 1, false});
            continue;
        }
        if (!XCAFDoc_ShapeTool::IsShape(frame.label)) {
            return BoundedProjectValidation::Invalid;
        }
        if (++definitionCount > kMaximumShapeDefinitions) {
            return BoundedProjectValidation::ResourceLimit;
        }
        const BoundedProjectValidation topology =
            ValidateBoundedTopologicalShape(
                XCAFDoc_ShapeTool::GetShape(frame.label),
                state,
                aggregateSubshapes);
        if (topology != BoundedProjectValidation::Valid) {
            return topology;
        }
        if (!XCAFDoc_ShapeTool::IsAssembly(frame.label)) {
            continue;
        }

        std::vector<TDF_Label> components;
        for (TDF_ChildIterator child(frame.label, Standard_False);
             child.More(); child.Next()) {
            const TDF_Label& component = child.Value();
            if (!XCAFDoc_ShapeTool::IsComponent(component)) {
                continue;
            }
            const Standard_Size componentCount =
                static_cast<Standard_Size>(components.size()) + 1;
            if (traversalNodes > kMaximumAssemblyTraversalNodes
                || componentCount
                    > kMaximumAssemblyTraversalNodes - traversalNodes
                || stack.size()
                    > static_cast<std::size_t>(
                        kMaximumAssemblyTraversalNodes)
                || componentCount
                    > kMaximumAssemblyTraversalNodes
                        - static_cast<Standard_Size>(stack.size())) {
                return BoundedProjectValidation::ResourceLimit;
            }
            components.push_back(component);
        }
        for (auto component = components.rbegin();
             component != components.rend(); ++component) {
            stack.push_back({*component, frame.depth + 1, false});
        }
    }
    return BoundedProjectValidation::Valid;
}

ImportedDocumentValidation ValidateImportedDocument(
    const Handle(OcctDocument)& document,
    const std::shared_ptr<NativeImportState>& state,
    const bool requireIdentifiers,
    std::size_t& leafCount) {
    ThrowIfCancelled(state);
    if (document.IsNull()
        || document->Document().IsNull()
        || document->Document()->HasOpenCommand()) {
        return ImportedDocumentValidation::Invalid;
    }
    const Handle(TDocStd_Document)& ocafDocument = document->Document();
    Standard_Real metersPerUnit = 0.0;
    if (!XCAFDoc_DocumentTool::GetLengthUnit(
            ocafDocument, metersPerUnit)
        || !std::isfinite(metersPerUnit)
        || metersPerUnit <= 0.0) {
        return ImportedDocumentValidation::Invalid;
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(ocafDocument->Main());
    if (shapeTool.IsNull()) {
        return ImportedDocumentValidation::Invalid;
    }
    const BoundedProjectValidation editorCompatibility =
        ValidateEditorCompatibleShapeTree(ocafDocument, state);
    if (editorCompatibility == BoundedProjectValidation::ResourceLimit) {
        return ImportedDocumentValidation::ResourceLimit;
    }
    if (editorCompatibility != BoundedProjectValidation::Valid) {
        return ImportedDocumentValidation::Invalid;
    }
    if (!document->ValidateGeometryRepresentations()) {
        return ImportedDocumentValidation::Invalid;
    }
    TDF_LabelSequence roots;
    shapeTool->GetFreeShapes(roots);
    if (roots.IsEmpty()) {
        return ImportedDocumentValidation::NoGeometry;
    }
    if (roots.Length() > kMaximumRoots) {
        return ImportedDocumentValidation::ResourceLimit;
    }
    if (requireIdentifiers && document->DocumentIdentifier().empty()) {
        return ImportedDocumentValidation::MissingIdentifiers;
    }

    leafCount = 0;
    std::size_t labelInstanceMappingCount = 0;
    XCAFPrs_DocumentExplorer explorer(
        ocafDocument,
        XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes);
    for (; explorer.More(); explorer.Next()) {
        ThrowIfCancelled(state);
        if (++leafCount > core3d::limits::kMaximumLeafPresentations) {
            return ImportedDocumentValidation::ResourceLimit;
        }
        const Standard_Integer currentDepth = explorer.CurrentDepth();
        if (currentDepth < 0
            || currentDepth > kMaximumOccurrenceDepth
            || labelInstanceMappingCount
                > kMaximumLabelInstanceMappings
                    - static_cast<std::size_t>(currentDepth + 1)) {
            return ImportedDocumentValidation::ResourceLimit;
        }
        labelInstanceMappingCount +=
            static_cast<std::size_t>(currentDepth + 1);
        for (Standard_Integer depth = 0;
             depth <= currentDepth; ++depth) {
            const TDF_Label& pathLabel = explorer.Current(depth).Label;
            if (pathLabel.IsNull()
                || pathLabel.Data() != ocafDocument->GetData()) {
                return ImportedDocumentValidation::Invalid;
            }
            if (requireIdentifiers
                && document->EntityIdentifierForLabel(pathLabel).empty()) {
                return ImportedDocumentValidation::MissingIdentifiers;
            }
        }
        const XCAFPrs_DocumentNode& node = explorer.Current();
        const TDF_Label definition = node.RefLabel.IsNull()
            ? node.Label
            : node.RefLabel;
        if (node.Label.IsNull()
            || definition.IsNull()
            || definition.Data() != ocafDocument->GetData()
            || !XCAFDoc_ShapeTool::IsShape(definition)) {
            return ImportedDocumentValidation::Invalid;
        }
        const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(definition);
        if (shape.IsNull()) {
            return ImportedDocumentValidation::Invalid;
        }
        if (requireIdentifiers
            && document->DefinitionIdentifierForLabel(definition).empty()) {
            return ImportedDocumentValidation::MissingIdentifiers;
        }
    }
    if (leafCount == 0) {
        return ImportedDocumentValidation::NoGeometry;
    }
    Standard_Integer materialCount = 0;
    if (XCAFDoc_DocumentTool::CheckVisMaterialTool(
            ocafDocument->Main())) {
        const Handle(XCAFDoc_VisMaterialTool) materialTool =
            XCAFDoc_DocumentTool::VisMaterialTool(ocafDocument->Main());
        if (materialTool.IsNull()) {
            return ImportedDocumentValidation::Invalid;
        }
        TDF_LabelSequence materials;
        materialTool->GetMaterials(materials);
        materialCount = materials.Length();
        if (materialCount > kMaximumMaterialDefinitions) {
            return ImportedDocumentValidation::ResourceLimit;
        }
    }
    if (XCAFDoc_DocumentTool::CheckMaterialTool(
            ocafDocument->Main())) {
        const Handle(XCAFDoc_MaterialTool) materialTool =
            XCAFDoc_DocumentTool::MaterialTool(ocafDocument->Main());
        if (materialTool.IsNull()) {
            return ImportedDocumentValidation::Invalid;
        }
        TDF_LabelSequence materials;
        materialTool->GetMaterialLabels(materials);
        if (materials.Length() > kMaximumMaterialDefinitions - materialCount) {
            return ImportedDocumentValidation::ResourceLimit;
        }
        materialCount += materials.Length();
    }
    if (materialCount > kMaximumMaterialDefinitions) {
        return ImportedDocumentValidation::ResourceLimit;
    }
    return ImportedDocumentValidation::Valid;
}

bool IsExactSingleArtifact(
    const std::shared_ptr<NativeImportState>& state) {
    struct stat info = {};
    if (::lstat(state->primaryPath.c_str(), &info) != 0
        || !S_ISREG(info.st_mode)
        || info.st_size <= 0
        || static_cast<std::uint64_t>(info.st_size)
            > kMaximumArtifactBytes) {
        return false;
    }
    const std::filesystem::path root(state->packageRootPath);
    const std::filesystem::path primary =
        std::filesystem::path(state->primaryPath).lexically_normal();
    std::error_code error;
    std::size_t count = 0;
    for (std::filesystem::directory_iterator iterator(root, error), end;
         !error && iterator != end;
         iterator.increment(error)) {
        ThrowIfCancelled(state);
        const std::filesystem::file_status status =
            iterator->symlink_status(error);
        if (error
            || std::filesystem::is_symlink(status)
            || !std::filesystem::is_regular_file(status)
            || iterator->path().lexically_normal() != primary) {
            return false;
        }
        ++count;
    }
    return !error && count == 1;
}

void ImportSTEP(
    const std::shared_ptr<NativeImportState>& state,
    const Handle(OcctDocument)& document,
    const PinnedStagedFile& stagedFile,
    const Message_ProgressRange& progress) {
    Message_ProgressScope scope(progress, "STEP import", 4);
    document->InitDoc();
    if (document->Document().IsNull()
        || document->Document()->HasOpenCommand()) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInternalFailure,
            "The private import document could not be created.");
    }
    scope.Next(1).Close();
    ThrowIfCancelled(state);

    {
        std::unique_lock<std::mutex> exchangeLock(
            Core3DSTEPExchangeMutex(), std::defer_lock);
        while (!exchangeLock.try_lock()) {
            ThrowIfCancelled(state);
            ::usleep(1'000);
        }
        ThrowIfCancelled(state);
        STEPCAFControl_Reader reader;
        reader.SetColorMode(Standard_True);
        reader.SetNameMode(Standard_True);
        // Physical STEP materials are intentionally excluded until the
        // sandbox-safe BinXCAF schema has a bounded material driver. Visual
        // surface colors remain enabled and survive the project handoff.
        reader.SetMatMode(Standard_False);
        reader.SetLayerMode(Standard_False);
        reader.SetPropsMode(Standard_False);
        reader.SetSHUOMode(Standard_False);
        reader.SetGDTMode(Standard_False);
        reader.SetViewMode(Standard_False);

        StepData_ConfParameters parameters;
        parameters.ReadColor = true;
        parameters.ReadName = true;
        parameters.ReadLayer = false;
        parameters.ReadProps = false;
        parameters.ReadSubshapeNames = false;
        parameters.ReadConstrRelation = false;
        parameters.ReadNonmanifold = false;
        parameters.ReadTessellated =
            StepData_ConfParameters::RWMode_Tessellated_Off;
        CancellableFileStreamBuffer streamBuffer(
            stagedFile.descriptor.Get(),
            state,
            stagedFile.identity,
            stagedFile.digest);
        if (!streamBuffer.IsOpen()) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInternalFailure,
                "The private STEP staging descriptor is unavailable.");
        }
        std::istream input(&streamBuffer);
        const IFSelect_ReturnStatus readStatus =
            reader.ChangeReader().ReadStream(
                state->stagedSourcePath.c_str(), parameters, input);
        ThrowIfCancelled(state);
        const bool streamValidated = streamBuffer.FinishValidation();
        ThrowIfCancelled(state);
        if (streamBuffer.ReadFailed()
            || !streamValidated
            || readStatus != IFSelect_RetDone) {
            throw NativeImportFailure(
                Core3DNativeImportErrorReadFailed,
                "The selected file is not a readable STEP model.");
        }
        ThrowIfCancelled(state);
        const Handle(StepData_StepModel) stepModel =
            reader.ChangeReader().StepModel();
        const Handle(XSControl_WorkSession) session =
            reader.ChangeReader().WS();
        const Interface_CheckIterator modelChecks = session.IsNull()
            ? Interface_CheckIterator()
            : session->ModelCheckList(Standard_True);
        if (stepModel.IsNull()
            || session.IsNull()
            || stepModel->NbEntities() <= 0
            || reader.NbRootsForTransfer() <= 0
            || !modelChecks.IsEmpty(Standard_True)) {
            throw NativeImportFailure(
                Core3DNativeImportErrorReadFailed,
                "The STEP model is structurally invalid.");
        }
        if (stepModel->NbEntities() > kMaximumSTEPEntities
            || reader.NbRootsForTransfer() > kMaximumRoots) {
            throw NativeImportFailure(
                Core3DNativeImportErrorResourceLimit,
                "The STEP model is too complex for a mobile project.");
        }
        STEPConstruct_ExternRefs externalReferences(session);
        externalReferences.LoadExternRefs();
        if (externalReferences.NbExternRefs() > 0
            || !reader.ExternFiles().IsEmpty()) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInvalidSource,
                "Linked STEP assemblies are not supported; choose a self-contained file.");
        }
        if (!reader.Transfer(document->ChangeDocument(), scope.Next(1))) {
            ThrowIfCancelled(state);
            throw NativeImportFailure(
                Core3DNativeImportErrorTransferFailed,
                "The STEP geometry could not be converted into an editable project.");
        }
        ThrowIfCancelled(state);
        const Handle(XSControl_TransferReader)& transferReader =
            session->TransferReader();
        const Interface_CheckIterator transferChecks = transferReader.IsNull()
            ? Interface_CheckIterator()
            : transferReader->LastCheckList();
        if (!reader.ExternFiles().IsEmpty()) {
            throw NativeImportFailure(
                Core3DNativeImportErrorInvalidSource,
                "Linked STEP assemblies are not supported; choose a self-contained file.");
        }
        if (!transferChecks.IsEmpty(Standard_True)) {
            throw NativeImportFailure(
                Core3DNativeImportErrorTransferFailed,
                "The STEP model contains geometry that could not be converted safely.");
        }
    }
    ThrowIfCancelled(state);

    std::size_t leafCount = 0;
    const ImportedDocumentValidation initialValidation =
        ValidateImportedDocument(document, state, false, leafCount);
    if (initialValidation == ImportedDocumentValidation::NoGeometry) {
        throw NativeImportFailure(
            Core3DNativeImportErrorNoGeometry,
            "The STEP model has no supported geometry.");
    }
    if (initialValidation == ImportedDocumentValidation::ResourceLimit) {
        throw NativeImportFailure(
            Core3DNativeImportErrorResourceLimit,
            "The STEP model exceeds mobile project limits.");
    }
    if (initialValidation != ImportedDocumentValidation::Valid) {
        throw NativeImportFailure(
            Core3DNativeImportErrorTransferFailed,
            "The converted STEP project is structurally invalid.");
    }
    scope.Next(1).Close();
    if (!document->MigrateLegacyIdentifiers()) {
        throw NativeImportFailure(
            Core3DNativeImportErrorMigrationFailed,
            "The imported model could not be prepared for editing.");
    }
    if (!document->MarkImportedBRepDefinitions()) {
        throw NativeImportFailure(
            Core3DNativeImportErrorMigrationFailed,
            "The imported model geometry could not be prepared for editing.");
    }
    std::size_t migratedLeafCount = 0;
    const ImportedDocumentValidation migratedValidation =
        ValidateImportedDocument(document, state, true, migratedLeafCount);
    if (migratedValidation == ImportedDocumentValidation::ResourceLimit) {
        throw NativeImportFailure(
            Core3DNativeImportErrorResourceLimit,
            "The STEP model exceeds mobile project limits.");
    }
    if (migratedValidation != ImportedDocumentValidation::Valid
        || migratedLeafCount != leafCount) {
        throw NativeImportFailure(
            Core3DNativeImportErrorMigrationFailed,
            "The imported model failed editable-project validation.");
    }
    state->shapeCount = leafCount;
    scope.Next(1).Close();
}

void SerializeAndValidate(
    const std::shared_ptr<NativeImportState>& state,
    const Handle(OcctDocument)& document,
    const Message_ProgressRange& progress) {
    Message_ProgressScope scope(progress, "Project handoff", 3);
    const std::string hiddenBase = (
        std::filesystem::path(state->packageRootPath)
        / "imported_project.tmp").string();
    const std::string savedPath = document->save(
        hiddenBase, scope.Next(1));
    ThrowIfCancelled(state);
    if (savedPath.empty()
        || std::filesystem::path(savedPath).parent_path()
            != std::filesystem::path(state->packageRootPath)
        || std::filesystem::path(savedPath).extension() != ".xbf") {
        throw NativeImportFailure(
            Core3DNativeImportErrorWriterFailed,
            "The editable project handoff could not be serialized.");
    }
    ThrowIfCancelled(state);

    Handle(OcctDocument) verifier = new OcctDocument();
    PrivateDocumentGuard verifierGuard(verifier);
    const bool didOpenVerifier = verifier->OpenPrivateExportSnapshot(
        savedPath, scope.Next(1));
    ThrowIfCancelled(state);
    if (!didOpenVerifier) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidArtifact,
            "The imported project could not be reopened safely.");
    }
    std::size_t verifiedLeafCount = 0;
    const ImportedDocumentValidation validation = ValidateImportedDocument(
        verifier, state, true, verifiedLeafCount);
    verifierGuard.Close();
    verifier.Nullify();
    if (validation != ImportedDocumentValidation::Valid
        || verifiedLeafCount != state->shapeCount) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidArtifact,
            "The imported project failed final validation.");
    }
    scope.Next(1).Close();
    ThrowIfCancelled(state);

    std::error_code renameError;
    std::filesystem::rename(
        std::filesystem::path(savedPath),
        std::filesystem::path(state->primaryPath),
        renameError);
    if (renameError || !IsExactSingleArtifact(state)) {
        throw NativeImportFailure(
            Core3DNativeImportErrorInvalidArtifact,
            "The imported project could not be finalized safely.");
    }
    ThrowIfCancelled(state);
}

NativeImportResult RunNativeImport(
    const std::shared_ptr<NativeImportState>& state) noexcept {
    NativeImportResult result;
    Handle(OcctDocument) document;
    try {
        OCC_CATCH_SIGNALS
        ThrowIfCancelled(state);
        switch (state->format) {
            case Core3DNativeImportFormatSTEP: {
                PinnedStagedFile stagedFile =
                    CopySourceIntoPrivateStaging(state);
                PreflightStagedSTEP(state, stagedFile);
                ThrowIfCancelled(state);

                Handle(Message_ProgressIndicator) progress =
                    new NativeImportProgress(&state->cancelled);
                Message_ProgressScope whole(
                    progress->Start(), "Native STEP import", 8);
                document = new OcctDocument();
                ImportSTEP(state, document, stagedFile, whole.Next(4));
                SerializeAndValidate(state, document, whole.Next(4));
                break;
            }
            default:
                throw NativeImportFailure(
                    Core3DNativeImportErrorInvalidSource,
                    "The selected import format is unsupported.");
        }
        ThrowIfCancelled(state);
        RemoveTreeNoThrow(state->stagingDirectoryPath);
        result.succeeded = true;
    } catch (const NativeImportFailure& failure) {
        result.errorCode = failure.Code();
        result.message = failure.what();
    } catch (const Standard_Failure& failure) {
        result.errorCode = state->cancelled.load(std::memory_order_acquire)
            ? Core3DNativeImportErrorCancelled
            : Core3DNativeImportErrorInternalFailure;
        result.message = failure.GetMessageString();
    } catch (const std::bad_alloc&) {
        result.errorCode = Core3DNativeImportErrorResourceLimit;
        result.message = "The STEP import exceeded available memory.";
    } catch (const std::exception& exception) {
        result.errorCode = state->cancelled.load(std::memory_order_acquire)
            ? Core3DNativeImportErrorCancelled
            : Core3DNativeImportErrorInternalFailure;
        result.message = exception.what();
    } catch (...) {
        result.errorCode = state->cancelled.load(std::memory_order_acquire)
            ? Core3DNativeImportErrorCancelled
            : Core3DNativeImportErrorInternalFailure;
        result.message = "The STEP import failed unexpectedly.";
    }

    if (!document.IsNull()) {
        document->ClosePrivateExportSnapshot();
        document.Nullify();
    }
    if (!result.succeeded) {
        RemoveTreeNoThrow(state->cleanupPath);
    }
    return result;
}

NSString *ErrorDescription(const NativeImportResult& result) {
    if (!result.message.empty()) {
        return [NSString stringWithUTF8String:result.message.c_str()]
            ?: @"The STEP import failed.";
    }
    return @"The STEP import failed.";
}

} // namespace

@interface Core3DNativeImportArtifact ()

@property(nonatomic, readwrite, copy) NSURL *primaryURL;
@property(nonatomic, readwrite, copy) NSURL *packageRootURL;
@property(nonatomic, readwrite, copy) NSURL *cleanupURL;
@property(nonatomic, readwrite) NSUInteger shapeCount;

- (instancetype)initWithPrimaryURL:(NSURL *)primaryURL
                     packageRootURL:(NSURL *)packageRootURL
                         cleanupURL:(NSURL *)cleanupURL
                         shapeCount:(NSUInteger)shapeCount;

@end

@implementation Core3DNativeImportArtifact

- (instancetype)initWithPrimaryURL:(NSURL *)primaryURL
                     packageRootURL:(NSURL *)packageRootURL
                         cleanupURL:(NSURL *)cleanupURL
                         shapeCount:(NSUInteger)shapeCount {
    self = [super init];
    if (self) {
        _primaryURL = [primaryURL copy];
        _packageRootURL = [packageRootURL copy];
        _cleanupURL = [cleanupURL copy];
        _shapeCount = shapeCount;
    }
    return self;
}

@end

@interface Core3DNativeImportOperation () {
    std::shared_ptr<NativeImportState> _state;
    NSURL *_sourceURL;
    Core3DNativeImportFormat _format;
}
@end

@implementation Core3DNativeImportOperation

- (instancetype)initWithSTEPURL:(NSURL *)stepURL {
    return [self initWithSourceURL:stepURL
                           format:Core3DNativeImportFormatSTEP];
}

- (instancetype)initWithSourceURL:(NSURL *)sourceURL
                            format:(Core3DNativeImportFormat)format {
    NSURL *temporaryRoot = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"Core3DNativeImport-%@", NSUUID.UUID.UUIDString]
        isDirectory:YES];
    return [self initWithSourceURL:sourceURL
                           format:format
                    temporaryRoot:temporaryRoot];
}

- (instancetype)initWithSourceURL:(NSURL *)sourceURL
                            format:(Core3DNativeImportFormat)format
                     temporaryRoot:(NSURL *)temporaryRoot {
    self = [super init];
    if (!self
        || format != Core3DNativeImportFormatSTEP
        || !sourceURL.isFileURL
        || !temporaryRoot.isFileURL) {
        return nil;
    }
    const char *sourcePath = sourceURL.path.fileSystemRepresentation;
    const char *cleanupPath = temporaryRoot.path.fileSystemRepresentation;
    if (sourcePath == nullptr || cleanupPath == nullptr) {
        return nil;
    }
    NSURL *stagingDirectory = [temporaryRoot
        URLByAppendingPathComponent:@"source" isDirectory:YES];
    NSURL *packageRoot = [temporaryRoot
        URLByAppendingPathComponent:@"contents" isDirectory:YES];
    NSError *directoryError = nil;
    if (![NSFileManager.defaultManager createDirectoryAtURL:stagingDirectory
                                withIntermediateDirectories:YES
                                                 attributes:nil
                                                      error:&directoryError]
        || ![NSFileManager.defaultManager createDirectoryAtURL:packageRoot
                                   withIntermediateDirectories:YES
                                                    attributes:nil
                                                         error:&directoryError]) {
        [NSFileManager.defaultManager removeItemAtURL:temporaryRoot error:nil];
        return nil;
    }

    _sourceURL = [sourceURL copy];
    _format = format;
    _state = std::make_shared<NativeImportState>();
    _state->sourcePath = sourcePath;
    _state->format = format;
    _state->cleanupPath = cleanupPath;
    _state->stagingDirectoryPath =
        stagingDirectory.path.fileSystemRepresentation;
    _state->stagedSourcePath =
        [stagingDirectory URLByAppendingPathComponent:@"model.step"]
            .path.fileSystemRepresentation;
    _state->packageRootPath = packageRoot.path.fileSystemRepresentation;
    _state->primaryPath =
        [packageRoot URLByAppendingPathComponent:@"project.xbf"]
            .path.fileSystemRepresentation;
    return self;
}

- (BOOL)isCancelled {
    return _state != nullptr
        && _state->cancelled.load(std::memory_order_acquire);
}

- (void)dealloc {
    const std::shared_ptr<NativeImportState> state = _state;
    if (state == nullptr) {
        return;
    }
    bool shouldCleanup = false;
    {
        std::lock_guard<std::mutex> lock(state->lifecycleMutex);
        if (!state->started) {
            state->started = true;
            state->finished = true;
            shouldCleanup = true;
        } else if (!state->finished) {
            state->cancelled.store(true, std::memory_order_release);
        }
    }
#ifdef DEBUG
    gDebugWorkerPauseCondition.notify_all();
    gDebugSourceIdentityCondition.notify_all();
    gDebugStructuralValidationCondition.notify_all();
#endif
    if (shouldCleanup) {
        dispatch_async(NativeImportCleanupQueue(), ^{
            RemoveTreeNoThrow(state->cleanupPath);
        });
    }
}

- (void)startWithCompletion:(Core3DNativeImportCompletion)completion {
    if (completion == nil) {
        return;
    }
    const std::shared_ptr<NativeImportState> state = _state;
    if (state == nullptr) {
        dispatch_async(dispatch_get_main_queue(), ^{
            completion(nil, [NSError
                errorWithDomain:Core3DNativeImportErrorDomain
                code:Core3DNativeImportErrorInternalFailure
                userInfo:@{
                    NSLocalizedDescriptionKey:
                        @"The STEP import operation is unavailable."
                }]);
        });
        return;
    }
    {
        std::lock_guard<std::mutex> lock(state->lifecycleMutex);
        if (state->started) {
            dispatch_async(dispatch_get_main_queue(), ^{
                completion(nil, [NSError
                    errorWithDomain:Core3DNativeImportErrorDomain
                    code:Core3DNativeImportErrorInternalFailure
                    userInfo:@{
                        NSLocalizedDescriptionKey:
                            @"The STEP import operation was already started."
                    }]);
            });
            return;
        }
        state->started = true;
    }

    Core3DNativeImportCompletion completionCopy = [completion copy];
    NSURL *sourceURL = [_sourceURL copy];
    [NativeImportQueue() addOperationWithBlock:^{
        WaitForDebugWorkerBarrier(state);
        const BOOL accessedSecurityScope =
            [sourceURL startAccessingSecurityScopedResource];
        NativeImportResult result = RunNativeImport(state);
        if (accessedSecurityScope) {
            [sourceURL stopAccessingSecurityScopedResource];
        }
        bool shouldDiscardArtifact = false;
        {
            std::lock_guard<std::mutex> lock(state->lifecycleMutex);
            if (result.succeeded
                && state->cancelled.load(std::memory_order_acquire)) {
                result.succeeded = false;
                result.errorCode = Core3DNativeImportErrorCancelled;
                result.message = "The STEP import was cancelled.";
                shouldDiscardArtifact = true;
            }
            state->finished = true;
        }
        if (shouldDiscardArtifact) {
            RemoveTreeNoThrow(state->cleanupPath);
        }
        dispatch_async(dispatch_get_main_queue(), ^{
            if (result.succeeded) {
                NSURL *primaryURL = [NSURL fileURLWithPath:
                    [NSString stringWithUTF8String:
                        state->primaryPath.c_str()]];
                NSURL *packageRootURL = [NSURL fileURLWithPath:
                    [NSString stringWithUTF8String:
                        state->packageRootPath.c_str()]];
                NSURL *cleanupURL = [NSURL fileURLWithPath:
                    [NSString stringWithUTF8String:
                        state->cleanupPath.c_str()]];
                Core3DNativeImportArtifact *artifact =
                    [[Core3DNativeImportArtifact alloc]
                        initWithPrimaryURL:primaryURL
                        packageRootURL:packageRootURL
                        cleanupURL:cleanupURL
                        shapeCount:state->shapeCount];
                completionCopy(artifact, nil);
                return;
            }
            NSError *error = [NSError
                errorWithDomain:Core3DNativeImportErrorDomain
                code:result.errorCode
                userInfo:@{
                    NSLocalizedDescriptionKey: ErrorDescription(result)
                }];
            completionCopy(nil, error);
        });
    }];
}

- (void)cancel {
    const std::shared_ptr<NativeImportState> state = _state;
    if (state == nullptr) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(state->lifecycleMutex);
        if (state->finished) {
            return;
        }
        state->cancelled.store(true, std::memory_order_release);
    }
#ifdef DEBUG
    gDebugWorkerPauseCondition.notify_all();
    gDebugSourceIdentityCondition.notify_all();
    gDebugStructuralValidationCondition.notify_all();
#endif
}

#ifdef DEBUG
+ (void)debugSetWorkerPaused:(BOOL)paused {
    {
        std::lock_guard<std::mutex> lock(gDebugWorkerPauseMutex);
        gDebugWorkerPaused = paused;
    }
    if (!paused) {
        gDebugWorkerPauseCondition.notify_all();
    }
}

+ (void)debugSetSourceIdentityBarrierPaused:(BOOL)paused {
    {
        std::lock_guard<std::mutex> lock(gDebugSourceIdentityMutex);
        gDebugSourceIdentityPaused = paused;
    }
    if (!paused) {
        gDebugSourceIdentityCondition.notify_all();
    }
}

+ (BOOL)debugIsSourceIdentityBarrierWaiting {
    std::lock_guard<std::mutex> lock(gDebugSourceIdentityMutex);
    return gDebugSourceIdentityWaiting;
}

+ (void)debugSetStructuralValidationBarrierPaused:(BOOL)paused {
    {
        std::lock_guard<std::mutex> lock(
            gDebugStructuralValidationMutex);
        if (paused && !gDebugStructuralValidationPaused) {
            gDebugStructuralValidationClaimed = false;
        }
        gDebugStructuralValidationPaused = paused;
        if (!paused) {
            gDebugStructuralValidationClaimed = false;
        }
    }
    if (!paused) {
        gDebugStructuralValidationCondition.notify_all();
    }
}

+ (BOOL)debugIsStructuralValidationBarrierWaiting {
    std::lock_guard<std::mutex> lock(gDebugStructuralValidationMutex);
    return gDebugStructuralValidationWaiting;
}
#endif

@end
