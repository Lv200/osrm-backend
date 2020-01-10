// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: rtree.proto

#ifndef PROTOBUF_INCLUDED_rtree_2eproto
#define PROTOBUF_INCLUDED_rtree_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3007000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3007001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_rtree_2eproto

// Internal implementation detail -- do not use these members.
struct TableStruct_rtree_2eproto {
  static const ::google::protobuf::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::ParseTable schema[5]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors_rtree_2eproto();
namespace pbrtree {
class LeafNode;
class LeafNodeDefaultTypeInternal;
extern LeafNodeDefaultTypeInternal _LeafNode_default_instance_;
class Leaves;
class LeavesDefaultTypeInternal;
extern LeavesDefaultTypeInternal _Leaves_default_instance_;
class Rectangle;
class RectangleDefaultTypeInternal;
extern RectangleDefaultTypeInternal _Rectangle_default_instance_;
class Segment;
class SegmentDefaultTypeInternal;
extern SegmentDefaultTypeInternal _Segment_default_instance_;
class Segments;
class SegmentsDefaultTypeInternal;
extern SegmentsDefaultTypeInternal _Segments_default_instance_;
}  // namespace pbrtree
namespace google {
namespace protobuf {
template<> ::pbrtree::LeafNode* Arena::CreateMaybeMessage<::pbrtree::LeafNode>(Arena*);
template<> ::pbrtree::Leaves* Arena::CreateMaybeMessage<::pbrtree::Leaves>(Arena*);
template<> ::pbrtree::Rectangle* Arena::CreateMaybeMessage<::pbrtree::Rectangle>(Arena*);
template<> ::pbrtree::Segment* Arena::CreateMaybeMessage<::pbrtree::Segment>(Arena*);
template<> ::pbrtree::Segments* Arena::CreateMaybeMessage<::pbrtree::Segments>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace pbrtree {

// ===================================================================

class Segment :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:pbrtree.Segment) */ {
 public:
  Segment();
  virtual ~Segment();

  Segment(const Segment& from);

  inline Segment& operator=(const Segment& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Segment(Segment&& from) noexcept
    : Segment() {
    *this = ::std::move(from);
  }

  inline Segment& operator=(Segment&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Segment& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Segment* internal_default_instance() {
    return reinterpret_cast<const Segment*>(
               &_Segment_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Segment* other);
  friend void swap(Segment& a, Segment& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Segment* New() const final {
    return CreateMaybeMessage<Segment>(nullptr);
  }

  Segment* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Segment>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Segment& from);
  void MergeFrom(const Segment& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Segment* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // uint32 u = 1;
  void clear_u();
  static const int kUFieldNumber = 1;
  ::google::protobuf::uint32 u() const;
  void set_u(::google::protobuf::uint32 value);

  // uint32 v = 2;
  void clear_v();
  static const int kVFieldNumber = 2;
  ::google::protobuf::uint32 v() const;
  void set_v(::google::protobuf::uint32 value);

  // uint32 forward_segment_id = 3;
  void clear_forward_segment_id();
  static const int kForwardSegmentIdFieldNumber = 3;
  ::google::protobuf::uint32 forward_segment_id() const;
  void set_forward_segment_id(::google::protobuf::uint32 value);

  // uint32 reverse_segment_id = 4;
  void clear_reverse_segment_id();
  static const int kReverseSegmentIdFieldNumber = 4;
  ::google::protobuf::uint32 reverse_segment_id() const;
  void set_reverse_segment_id(::google::protobuf::uint32 value);

  // uint32 forward_segment_position = 5;
  void clear_forward_segment_position();
  static const int kForwardSegmentPositionFieldNumber = 5;
  ::google::protobuf::uint32 forward_segment_position() const;
  void set_forward_segment_position(::google::protobuf::uint32 value);

  // bool forward_enabled = 6;
  void clear_forward_enabled();
  static const int kForwardEnabledFieldNumber = 6;
  bool forward_enabled() const;
  void set_forward_enabled(bool value);

  // bool reverse_enabled = 7;
  void clear_reverse_enabled();
  static const int kReverseEnabledFieldNumber = 7;
  bool reverse_enabled() const;
  void set_reverse_enabled(bool value);

  // @@protoc_insertion_point(class_scope:pbrtree.Segment)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 u_;
  ::google::protobuf::uint32 v_;
  ::google::protobuf::uint32 forward_segment_id_;
  ::google::protobuf::uint32 reverse_segment_id_;
  ::google::protobuf::uint32 forward_segment_position_;
  bool forward_enabled_;
  bool reverse_enabled_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_rtree_2eproto;
};
// -------------------------------------------------------------------

class Rectangle :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:pbrtree.Rectangle) */ {
 public:
  Rectangle();
  virtual ~Rectangle();

  Rectangle(const Rectangle& from);

  inline Rectangle& operator=(const Rectangle& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Rectangle(Rectangle&& from) noexcept
    : Rectangle() {
    *this = ::std::move(from);
  }

  inline Rectangle& operator=(Rectangle&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Rectangle& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Rectangle* internal_default_instance() {
    return reinterpret_cast<const Rectangle*>(
               &_Rectangle_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(Rectangle* other);
  friend void swap(Rectangle& a, Rectangle& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Rectangle* New() const final {
    return CreateMaybeMessage<Rectangle>(nullptr);
  }

  Rectangle* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Rectangle>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Rectangle& from);
  void MergeFrom(const Rectangle& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Rectangle* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // uint64 min_lat = 1;
  void clear_min_lat();
  static const int kMinLatFieldNumber = 1;
  ::google::protobuf::uint64 min_lat() const;
  void set_min_lat(::google::protobuf::uint64 value);

  // uint64 min_lon = 2;
  void clear_min_lon();
  static const int kMinLonFieldNumber = 2;
  ::google::protobuf::uint64 min_lon() const;
  void set_min_lon(::google::protobuf::uint64 value);

  // uint64 max_lat = 3;
  void clear_max_lat();
  static const int kMaxLatFieldNumber = 3;
  ::google::protobuf::uint64 max_lat() const;
  void set_max_lat(::google::protobuf::uint64 value);

  // uint64 max_lon = 4;
  void clear_max_lon();
  static const int kMaxLonFieldNumber = 4;
  ::google::protobuf::uint64 max_lon() const;
  void set_max_lon(::google::protobuf::uint64 value);

  // @@protoc_insertion_point(class_scope:pbrtree.Rectangle)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint64 min_lat_;
  ::google::protobuf::uint64 min_lon_;
  ::google::protobuf::uint64 max_lat_;
  ::google::protobuf::uint64 max_lon_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_rtree_2eproto;
};
// -------------------------------------------------------------------

class LeafNode :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:pbrtree.LeafNode) */ {
 public:
  LeafNode();
  virtual ~LeafNode();

  LeafNode(const LeafNode& from);

  inline LeafNode& operator=(const LeafNode& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  LeafNode(LeafNode&& from) noexcept
    : LeafNode() {
    *this = ::std::move(from);
  }

  inline LeafNode& operator=(LeafNode&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const LeafNode& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LeafNode* internal_default_instance() {
    return reinterpret_cast<const LeafNode*>(
               &_LeafNode_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(LeafNode* other);
  friend void swap(LeafNode& a, LeafNode& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline LeafNode* New() const final {
    return CreateMaybeMessage<LeafNode>(nullptr);
  }

  LeafNode* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<LeafNode>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const LeafNode& from);
  void MergeFrom(const LeafNode& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(LeafNode* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .pbrtree.Rectangle minimum_bounding_rectangle = 1;
  bool has_minimum_bounding_rectangle() const;
  void clear_minimum_bounding_rectangle();
  static const int kMinimumBoundingRectangleFieldNumber = 1;
  const ::pbrtree::Rectangle& minimum_bounding_rectangle() const;
  ::pbrtree::Rectangle* release_minimum_bounding_rectangle();
  ::pbrtree::Rectangle* mutable_minimum_bounding_rectangle();
  void set_allocated_minimum_bounding_rectangle(::pbrtree::Rectangle* minimum_bounding_rectangle);

  // uint32 indexStart = 2;
  void clear_indexstart();
  static const int kIndexStartFieldNumber = 2;
  ::google::protobuf::uint32 indexstart() const;
  void set_indexstart(::google::protobuf::uint32 value);

  // uint32 indexEnd = 3;
  void clear_indexend();
  static const int kIndexEndFieldNumber = 3;
  ::google::protobuf::uint32 indexend() const;
  void set_indexend(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:pbrtree.LeafNode)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::pbrtree::Rectangle* minimum_bounding_rectangle_;
  ::google::protobuf::uint32 indexstart_;
  ::google::protobuf::uint32 indexend_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_rtree_2eproto;
};
// -------------------------------------------------------------------

class Leaves :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:pbrtree.Leaves) */ {
 public:
  Leaves();
  virtual ~Leaves();

  Leaves(const Leaves& from);

  inline Leaves& operator=(const Leaves& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Leaves(Leaves&& from) noexcept
    : Leaves() {
    *this = ::std::move(from);
  }

  inline Leaves& operator=(Leaves&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Leaves& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Leaves* internal_default_instance() {
    return reinterpret_cast<const Leaves*>(
               &_Leaves_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  void Swap(Leaves* other);
  friend void swap(Leaves& a, Leaves& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Leaves* New() const final {
    return CreateMaybeMessage<Leaves>(nullptr);
  }

  Leaves* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Leaves>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Leaves& from);
  void MergeFrom(const Leaves& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Leaves* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .pbrtree.LeafNode items = 1;
  int items_size() const;
  void clear_items();
  static const int kItemsFieldNumber = 1;
  ::pbrtree::LeafNode* mutable_items(int index);
  ::google::protobuf::RepeatedPtrField< ::pbrtree::LeafNode >*
      mutable_items();
  const ::pbrtree::LeafNode& items(int index) const;
  ::pbrtree::LeafNode* add_items();
  const ::google::protobuf::RepeatedPtrField< ::pbrtree::LeafNode >&
      items() const;

  // @@protoc_insertion_point(class_scope:pbrtree.Leaves)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::pbrtree::LeafNode > items_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_rtree_2eproto;
};
// -------------------------------------------------------------------

class Segments :
    public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:pbrtree.Segments) */ {
 public:
  Segments();
  virtual ~Segments();

  Segments(const Segments& from);

  inline Segments& operator=(const Segments& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Segments(Segments&& from) noexcept
    : Segments() {
    *this = ::std::move(from);
  }

  inline Segments& operator=(Segments&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor() {
    return default_instance().GetDescriptor();
  }
  static const Segments& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Segments* internal_default_instance() {
    return reinterpret_cast<const Segments*>(
               &_Segments_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    4;

  void Swap(Segments* other);
  friend void swap(Segments& a, Segments& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Segments* New() const final {
    return CreateMaybeMessage<Segments>(nullptr);
  }

  Segments* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Segments>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Segments& from);
  void MergeFrom(const Segments& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  static const char* _InternalParse(const char* begin, const char* end, void* object, ::google::protobuf::internal::ParseContext* ctx);
  ::google::protobuf::internal::ParseFunc _ParseFunc() const final { return _InternalParse; }
  #else
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Segments* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .pbrtree.Segment items = 1;
  int items_size() const;
  void clear_items();
  static const int kItemsFieldNumber = 1;
  ::pbrtree::Segment* mutable_items(int index);
  ::google::protobuf::RepeatedPtrField< ::pbrtree::Segment >*
      mutable_items();
  const ::pbrtree::Segment& items(int index) const;
  ::pbrtree::Segment* add_items();
  const ::google::protobuf::RepeatedPtrField< ::pbrtree::Segment >&
      items() const;

  // @@protoc_insertion_point(class_scope:pbrtree.Segments)
 private:
  class HasBitSetters;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::pbrtree::Segment > items_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_rtree_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Segment

// uint32 u = 1;
inline void Segment::clear_u() {
  u_ = 0u;
}
inline ::google::protobuf::uint32 Segment::u() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.u)
  return u_;
}
inline void Segment::set_u(::google::protobuf::uint32 value) {
  
  u_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.u)
}

// uint32 v = 2;
inline void Segment::clear_v() {
  v_ = 0u;
}
inline ::google::protobuf::uint32 Segment::v() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.v)
  return v_;
}
inline void Segment::set_v(::google::protobuf::uint32 value) {
  
  v_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.v)
}

// uint32 forward_segment_id = 3;
inline void Segment::clear_forward_segment_id() {
  forward_segment_id_ = 0u;
}
inline ::google::protobuf::uint32 Segment::forward_segment_id() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.forward_segment_id)
  return forward_segment_id_;
}
inline void Segment::set_forward_segment_id(::google::protobuf::uint32 value) {
  
  forward_segment_id_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.forward_segment_id)
}

// uint32 reverse_segment_id = 4;
inline void Segment::clear_reverse_segment_id() {
  reverse_segment_id_ = 0u;
}
inline ::google::protobuf::uint32 Segment::reverse_segment_id() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.reverse_segment_id)
  return reverse_segment_id_;
}
inline void Segment::set_reverse_segment_id(::google::protobuf::uint32 value) {
  
  reverse_segment_id_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.reverse_segment_id)
}

// uint32 forward_segment_position = 5;
inline void Segment::clear_forward_segment_position() {
  forward_segment_position_ = 0u;
}
inline ::google::protobuf::uint32 Segment::forward_segment_position() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.forward_segment_position)
  return forward_segment_position_;
}
inline void Segment::set_forward_segment_position(::google::protobuf::uint32 value) {
  
  forward_segment_position_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.forward_segment_position)
}

// bool forward_enabled = 6;
inline void Segment::clear_forward_enabled() {
  forward_enabled_ = false;
}
inline bool Segment::forward_enabled() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.forward_enabled)
  return forward_enabled_;
}
inline void Segment::set_forward_enabled(bool value) {
  
  forward_enabled_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.forward_enabled)
}

// bool reverse_enabled = 7;
inline void Segment::clear_reverse_enabled() {
  reverse_enabled_ = false;
}
inline bool Segment::reverse_enabled() const {
  // @@protoc_insertion_point(field_get:pbrtree.Segment.reverse_enabled)
  return reverse_enabled_;
}
inline void Segment::set_reverse_enabled(bool value) {
  
  reverse_enabled_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Segment.reverse_enabled)
}

// -------------------------------------------------------------------

// Rectangle

// uint64 min_lat = 1;
inline void Rectangle::clear_min_lat() {
  min_lat_ = PROTOBUF_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Rectangle::min_lat() const {
  // @@protoc_insertion_point(field_get:pbrtree.Rectangle.min_lat)
  return min_lat_;
}
inline void Rectangle::set_min_lat(::google::protobuf::uint64 value) {
  
  min_lat_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Rectangle.min_lat)
}

// uint64 min_lon = 2;
inline void Rectangle::clear_min_lon() {
  min_lon_ = PROTOBUF_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Rectangle::min_lon() const {
  // @@protoc_insertion_point(field_get:pbrtree.Rectangle.min_lon)
  return min_lon_;
}
inline void Rectangle::set_min_lon(::google::protobuf::uint64 value) {
  
  min_lon_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Rectangle.min_lon)
}

// uint64 max_lat = 3;
inline void Rectangle::clear_max_lat() {
  max_lat_ = PROTOBUF_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Rectangle::max_lat() const {
  // @@protoc_insertion_point(field_get:pbrtree.Rectangle.max_lat)
  return max_lat_;
}
inline void Rectangle::set_max_lat(::google::protobuf::uint64 value) {
  
  max_lat_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Rectangle.max_lat)
}

// uint64 max_lon = 4;
inline void Rectangle::clear_max_lon() {
  max_lon_ = PROTOBUF_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Rectangle::max_lon() const {
  // @@protoc_insertion_point(field_get:pbrtree.Rectangle.max_lon)
  return max_lon_;
}
inline void Rectangle::set_max_lon(::google::protobuf::uint64 value) {
  
  max_lon_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.Rectangle.max_lon)
}

// -------------------------------------------------------------------

// LeafNode

// .pbrtree.Rectangle minimum_bounding_rectangle = 1;
inline bool LeafNode::has_minimum_bounding_rectangle() const {
  return this != internal_default_instance() && minimum_bounding_rectangle_ != nullptr;
}
inline void LeafNode::clear_minimum_bounding_rectangle() {
  if (GetArenaNoVirtual() == nullptr && minimum_bounding_rectangle_ != nullptr) {
    delete minimum_bounding_rectangle_;
  }
  minimum_bounding_rectangle_ = nullptr;
}
inline const ::pbrtree::Rectangle& LeafNode::minimum_bounding_rectangle() const {
  const ::pbrtree::Rectangle* p = minimum_bounding_rectangle_;
  // @@protoc_insertion_point(field_get:pbrtree.LeafNode.minimum_bounding_rectangle)
  return p != nullptr ? *p : *reinterpret_cast<const ::pbrtree::Rectangle*>(
      &::pbrtree::_Rectangle_default_instance_);
}
inline ::pbrtree::Rectangle* LeafNode::release_minimum_bounding_rectangle() {
  // @@protoc_insertion_point(field_release:pbrtree.LeafNode.minimum_bounding_rectangle)
  
  ::pbrtree::Rectangle* temp = minimum_bounding_rectangle_;
  minimum_bounding_rectangle_ = nullptr;
  return temp;
}
inline ::pbrtree::Rectangle* LeafNode::mutable_minimum_bounding_rectangle() {
  
  if (minimum_bounding_rectangle_ == nullptr) {
    auto* p = CreateMaybeMessage<::pbrtree::Rectangle>(GetArenaNoVirtual());
    minimum_bounding_rectangle_ = p;
  }
  // @@protoc_insertion_point(field_mutable:pbrtree.LeafNode.minimum_bounding_rectangle)
  return minimum_bounding_rectangle_;
}
inline void LeafNode::set_allocated_minimum_bounding_rectangle(::pbrtree::Rectangle* minimum_bounding_rectangle) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete minimum_bounding_rectangle_;
  }
  if (minimum_bounding_rectangle) {
    ::google::protobuf::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      minimum_bounding_rectangle = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, minimum_bounding_rectangle, submessage_arena);
    }
    
  } else {
    
  }
  minimum_bounding_rectangle_ = minimum_bounding_rectangle;
  // @@protoc_insertion_point(field_set_allocated:pbrtree.LeafNode.minimum_bounding_rectangle)
}

// uint32 indexStart = 2;
inline void LeafNode::clear_indexstart() {
  indexstart_ = 0u;
}
inline ::google::protobuf::uint32 LeafNode::indexstart() const {
  // @@protoc_insertion_point(field_get:pbrtree.LeafNode.indexStart)
  return indexstart_;
}
inline void LeafNode::set_indexstart(::google::protobuf::uint32 value) {
  
  indexstart_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.LeafNode.indexStart)
}

// uint32 indexEnd = 3;
inline void LeafNode::clear_indexend() {
  indexend_ = 0u;
}
inline ::google::protobuf::uint32 LeafNode::indexend() const {
  // @@protoc_insertion_point(field_get:pbrtree.LeafNode.indexEnd)
  return indexend_;
}
inline void LeafNode::set_indexend(::google::protobuf::uint32 value) {
  
  indexend_ = value;
  // @@protoc_insertion_point(field_set:pbrtree.LeafNode.indexEnd)
}

// -------------------------------------------------------------------

// Leaves

// repeated .pbrtree.LeafNode items = 1;
inline int Leaves::items_size() const {
  return items_.size();
}
inline void Leaves::clear_items() {
  items_.Clear();
}
inline ::pbrtree::LeafNode* Leaves::mutable_items(int index) {
  // @@protoc_insertion_point(field_mutable:pbrtree.Leaves.items)
  return items_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::pbrtree::LeafNode >*
Leaves::mutable_items() {
  // @@protoc_insertion_point(field_mutable_list:pbrtree.Leaves.items)
  return &items_;
}
inline const ::pbrtree::LeafNode& Leaves::items(int index) const {
  // @@protoc_insertion_point(field_get:pbrtree.Leaves.items)
  return items_.Get(index);
}
inline ::pbrtree::LeafNode* Leaves::add_items() {
  // @@protoc_insertion_point(field_add:pbrtree.Leaves.items)
  return items_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::pbrtree::LeafNode >&
Leaves::items() const {
  // @@protoc_insertion_point(field_list:pbrtree.Leaves.items)
  return items_;
}

// -------------------------------------------------------------------

// Segments

// repeated .pbrtree.Segment items = 1;
inline int Segments::items_size() const {
  return items_.size();
}
inline void Segments::clear_items() {
  items_.Clear();
}
inline ::pbrtree::Segment* Segments::mutable_items(int index) {
  // @@protoc_insertion_point(field_mutable:pbrtree.Segments.items)
  return items_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::pbrtree::Segment >*
Segments::mutable_items() {
  // @@protoc_insertion_point(field_mutable_list:pbrtree.Segments.items)
  return &items_;
}
inline const ::pbrtree::Segment& Segments::items(int index) const {
  // @@protoc_insertion_point(field_get:pbrtree.Segments.items)
  return items_.Get(index);
}
inline ::pbrtree::Segment* Segments::add_items() {
  // @@protoc_insertion_point(field_add:pbrtree.Segments.items)
  return items_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::pbrtree::Segment >&
Segments::items() const {
  // @@protoc_insertion_point(field_list:pbrtree.Segments.items)
  return items_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pbrtree

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // PROTOBUF_INCLUDED_rtree_2eproto
