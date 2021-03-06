// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: command-packet.proto

#ifndef PROTOBUF_command_2dpacket_2eproto__INCLUDED
#define PROTOBUF_command_2dpacket_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2004001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_message_reflection.h>
// @@protoc_insertion_point(includes)

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_command_2dpacket_2eproto();
void protobuf_AssignDesc_command_2dpacket_2eproto();
void protobuf_ShutdownFile_command_2dpacket_2eproto();

class StrategyPacket;
class StrategyPacket_Tactic;

enum StrategyPacket_Play {
  StrategyPacket_Play_Stop = 0,
  StrategyPacket_Play_PositionGather = 1,
  StrategyPacket_Play_Offense1 = 2,
  StrategyPacket_Play_Offense2 = 3,
  StrategyPacket_Play_Defense1 = 4,
  StrategyPacket_Play_Defense2 = 5,
  StrategyPacket_Play_PositionOurKickoff = 6,
  StrategyPacket_Play_PositionOppKickoff = 7,
  StrategyPacket_Play_PositionOurFreeKick = 8,
  StrategyPacket_Play_PositionOppFreeKick = 9,
  StrategyPacket_Play_PositionOurFreeBall = 10,
  StrategyPacket_Play_PositionOppFreeBall = 11,
  StrategyPacket_Play_PositionOurPenalty = 12,
  StrategyPacket_Play_PositionOppPenalty = 13,
  StrategyPacket_Play_PositionOurGoalKick = 14,
  StrategyPacket_Play_PositionOppGoalKick = 15,
  StrategyPacket_Play_Kickoff = 16,
  StrategyPacket_Play_OurFreeKick = 17,
  StrategyPacket_Play_OppFreeKick = 18,
  StrategyPacket_Play_OurFreeBall = 19,
  StrategyPacket_Play_OppFreeBall = 20,
  StrategyPacket_Play_PenaltyOur = 21,
  StrategyPacket_Play_PenaltyOpp = 22,
  StrategyPacket_Play_OurGoalKick = 23,
  StrategyPacket_Play_OppGoalKick = 24,
  StrategyPacket_Play_TakeGoalKick = 25,
  StrategyPacket_Play_TestPlay = 26,
  StrategyPacket_Play_SetPosition = 27,
  StrategyPacket_Play_SuperOffense = 28,
  StrategyPacket_Play_SuperDefense = 29,
  StrategyPacket_Play_None = 30
};
bool StrategyPacket_Play_IsValid(int value);
const StrategyPacket_Play StrategyPacket_Play_Play_MIN = StrategyPacket_Play_Stop;
const StrategyPacket_Play StrategyPacket_Play_Play_MAX = StrategyPacket_Play_None;
const int StrategyPacket_Play_Play_ARRAYSIZE = StrategyPacket_Play_Play_MAX + 1;

const ::google::protobuf::EnumDescriptor* StrategyPacket_Play_descriptor();
inline const ::std::string& StrategyPacket_Play_Name(StrategyPacket_Play value) {
  return ::google::protobuf::internal::NameOfEnum(
    StrategyPacket_Play_descriptor(), value);
}
inline bool StrategyPacket_Play_Parse(
    const ::std::string& name, StrategyPacket_Play* value) {
  return ::google::protobuf::internal::ParseNamedEnum<StrategyPacket_Play>(
    StrategyPacket_Play_descriptor(), name, value);
}
enum StrategyPacket_TacticID {
  StrategyPacket_TacticID_Block = 0,
  StrategyPacket_TacticID_ChargeBall = 1,
  StrategyPacket_TacticID_CoverGoal = 2,
  StrategyPacket_TacticID_DragToGoal = 3,
  StrategyPacket_TacticID_DefendLine = 4,
  StrategyPacket_TacticID_DefendPoint = 5,
  StrategyPacket_TacticID_GoalieOur = 6,
  StrategyPacket_TacticID_GoalieOpp = 7,
  StrategyPacket_TacticID_MarkBot = 8,
  StrategyPacket_TacticID_Pass = 9,
  StrategyPacket_TacticID_Position = 11,
  StrategyPacket_TacticID_PositionForPenalty = 12,
  StrategyPacket_TacticID_PositionForStart = 13,
  StrategyPacket_TacticID_ReceiveBall = 14,
  StrategyPacket_TacticID_Defend = 15,
  StrategyPacket_TacticID_Attack = 16,
  StrategyPacket_TacticID_Steal = 17,
  StrategyPacket_TacticID_Velocity = 19
};
bool StrategyPacket_TacticID_IsValid(int value);
const StrategyPacket_TacticID StrategyPacket_TacticID_TacticID_MIN = StrategyPacket_TacticID_Block;
const StrategyPacket_TacticID StrategyPacket_TacticID_TacticID_MAX = StrategyPacket_TacticID_Velocity;
const int StrategyPacket_TacticID_TacticID_ARRAYSIZE = StrategyPacket_TacticID_TacticID_MAX + 1;

const ::google::protobuf::EnumDescriptor* StrategyPacket_TacticID_descriptor();
inline const ::std::string& StrategyPacket_TacticID_Name(StrategyPacket_TacticID value) {
  return ::google::protobuf::internal::NameOfEnum(
    StrategyPacket_TacticID_descriptor(), value);
}
inline bool StrategyPacket_TacticID_Parse(
    const ::std::string& name, StrategyPacket_TacticID* value) {
  return ::google::protobuf::internal::ParseNamedEnum<StrategyPacket_TacticID>(
    StrategyPacket_TacticID_descriptor(), name, value);
}
enum StrategyPacket_Which {
  StrategyPacket_Which_PLAY = 1,
  StrategyPacket_Which_TACTIC = 2
};
bool StrategyPacket_Which_IsValid(int value);
const StrategyPacket_Which StrategyPacket_Which_Which_MIN = StrategyPacket_Which_PLAY;
const StrategyPacket_Which StrategyPacket_Which_Which_MAX = StrategyPacket_Which_TACTIC;
const int StrategyPacket_Which_Which_ARRAYSIZE = StrategyPacket_Which_Which_MAX + 1;

const ::google::protobuf::EnumDescriptor* StrategyPacket_Which_descriptor();
inline const ::std::string& StrategyPacket_Which_Name(StrategyPacket_Which value) {
  return ::google::protobuf::internal::NameOfEnum(
    StrategyPacket_Which_descriptor(), value);
}
inline bool StrategyPacket_Which_Parse(
    const ::std::string& name, StrategyPacket_Which* value) {
  return ::google::protobuf::internal::ParseNamedEnum<StrategyPacket_Which>(
    StrategyPacket_Which_descriptor(), name, value);
}
// ===================================================================

class StrategyPacket_Tactic : public ::google::protobuf::Message {
 public:
  StrategyPacket_Tactic();
  virtual ~StrategyPacket_Tactic();
  
  StrategyPacket_Tactic(const StrategyPacket_Tactic& from);
  
  inline StrategyPacket_Tactic& operator=(const StrategyPacket_Tactic& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const StrategyPacket_Tactic& default_instance();
  
  void Swap(StrategyPacket_Tactic* other);
  
  // implements Message ----------------------------------------------
  
  StrategyPacket_Tactic* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const StrategyPacket_Tactic& from);
  void MergeFrom(const StrategyPacket_Tactic& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  // accessors -------------------------------------------------------
  
  // required .StrategyPacket.TacticID tID = 1;
  inline bool has_tid() const;
  inline void clear_tid();
  static const int kTIDFieldNumber = 1;
  inline ::StrategyPacket_TacticID tid() const;
  inline void set_tid(::StrategyPacket_TacticID value);
  
  // required int32 botID = 2;
  inline bool has_botid() const;
  inline void clear_botid();
  static const int kBotIDFieldNumber = 2;
  inline ::google::protobuf::int32 botid() const;
  inline void set_botid(::google::protobuf::int32 value);
  
  // @@protoc_insertion_point(class_scope:StrategyPacket.Tactic)
 private:
  inline void set_has_tid();
  inline void clear_has_tid();
  inline void set_has_botid();
  inline void clear_has_botid();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  int tid_;
  ::google::protobuf::int32 botid_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];
  
  friend void  protobuf_AddDesc_command_2dpacket_2eproto();
  friend void protobuf_AssignDesc_command_2dpacket_2eproto();
  friend void protobuf_ShutdownFile_command_2dpacket_2eproto();
  
  void InitAsDefaultInstance();
  static StrategyPacket_Tactic* default_instance_;
};
// -------------------------------------------------------------------

class StrategyPacket : public ::google::protobuf::Message {
 public:
  StrategyPacket();
  virtual ~StrategyPacket();
  
  StrategyPacket(const StrategyPacket& from);
  
  inline StrategyPacket& operator=(const StrategyPacket& from) {
    CopyFrom(from);
    return *this;
  }
  
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }
  
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }
  
  static const ::google::protobuf::Descriptor* descriptor();
  static const StrategyPacket& default_instance();
  
  void Swap(StrategyPacket* other);
  
  // implements Message ----------------------------------------------
  
  StrategyPacket* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const StrategyPacket& from);
  void MergeFrom(const StrategyPacket& from);
  void Clear();
  bool IsInitialized() const;
  
  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  
  ::google::protobuf::Metadata GetMetadata() const;
  
  // nested types ----------------------------------------------------
  
  typedef StrategyPacket_Tactic Tactic;
  
  typedef StrategyPacket_Play Play;
  static const Play Stop = StrategyPacket_Play_Stop;
  static const Play PositionGather = StrategyPacket_Play_PositionGather;
  static const Play Offense1 = StrategyPacket_Play_Offense1;
  static const Play Offense2 = StrategyPacket_Play_Offense2;
  static const Play Defense1 = StrategyPacket_Play_Defense1;
  static const Play Defense2 = StrategyPacket_Play_Defense2;
  static const Play PositionOurKickoff = StrategyPacket_Play_PositionOurKickoff;
  static const Play PositionOppKickoff = StrategyPacket_Play_PositionOppKickoff;
  static const Play PositionOurFreeKick = StrategyPacket_Play_PositionOurFreeKick;
  static const Play PositionOppFreeKick = StrategyPacket_Play_PositionOppFreeKick;
  static const Play PositionOurFreeBall = StrategyPacket_Play_PositionOurFreeBall;
  static const Play PositionOppFreeBall = StrategyPacket_Play_PositionOppFreeBall;
  static const Play PositionOurPenalty = StrategyPacket_Play_PositionOurPenalty;
  static const Play PositionOppPenalty = StrategyPacket_Play_PositionOppPenalty;
  static const Play PositionOurGoalKick = StrategyPacket_Play_PositionOurGoalKick;
  static const Play PositionOppGoalKick = StrategyPacket_Play_PositionOppGoalKick;
  static const Play Kickoff = StrategyPacket_Play_Kickoff;
  static const Play OurFreeKick = StrategyPacket_Play_OurFreeKick;
  static const Play OppFreeKick = StrategyPacket_Play_OppFreeKick;
  static const Play OurFreeBall = StrategyPacket_Play_OurFreeBall;
  static const Play OppFreeBall = StrategyPacket_Play_OppFreeBall;
  static const Play PenaltyOur = StrategyPacket_Play_PenaltyOur;
  static const Play PenaltyOpp = StrategyPacket_Play_PenaltyOpp;
  static const Play OurGoalKick = StrategyPacket_Play_OurGoalKick;
  static const Play OppGoalKick = StrategyPacket_Play_OppGoalKick;
  static const Play TakeGoalKick = StrategyPacket_Play_TakeGoalKick;
  static const Play TestPlay = StrategyPacket_Play_TestPlay;
  static const Play SetPosition = StrategyPacket_Play_SetPosition;
  static const Play SuperOffense = StrategyPacket_Play_SuperOffense;
  static const Play SuperDefense = StrategyPacket_Play_SuperDefense;
  static const Play None = StrategyPacket_Play_None;
  static inline bool Play_IsValid(int value) {
    return StrategyPacket_Play_IsValid(value);
  }
  static const Play Play_MIN =
    StrategyPacket_Play_Play_MIN;
  static const Play Play_MAX =
    StrategyPacket_Play_Play_MAX;
  static const int Play_ARRAYSIZE =
    StrategyPacket_Play_Play_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Play_descriptor() {
    return StrategyPacket_Play_descriptor();
  }
  static inline const ::std::string& Play_Name(Play value) {
    return StrategyPacket_Play_Name(value);
  }
  static inline bool Play_Parse(const ::std::string& name,
      Play* value) {
    return StrategyPacket_Play_Parse(name, value);
  }
  
  typedef StrategyPacket_TacticID TacticID;
  static const TacticID Block = StrategyPacket_TacticID_Block;
  static const TacticID ChargeBall = StrategyPacket_TacticID_ChargeBall;
  static const TacticID CoverGoal = StrategyPacket_TacticID_CoverGoal;
  static const TacticID DragToGoal = StrategyPacket_TacticID_DragToGoal;
  static const TacticID DefendLine = StrategyPacket_TacticID_DefendLine;
  static const TacticID DefendPoint = StrategyPacket_TacticID_DefendPoint;
  static const TacticID GoalieOur = StrategyPacket_TacticID_GoalieOur;
  static const TacticID GoalieOpp = StrategyPacket_TacticID_GoalieOpp;
  static const TacticID MarkBot = StrategyPacket_TacticID_MarkBot;
  static const TacticID Pass = StrategyPacket_TacticID_Pass;
  static const TacticID Position = StrategyPacket_TacticID_Position;
  static const TacticID PositionForPenalty = StrategyPacket_TacticID_PositionForPenalty;
  static const TacticID PositionForStart = StrategyPacket_TacticID_PositionForStart;
  static const TacticID ReceiveBall = StrategyPacket_TacticID_ReceiveBall;
  static const TacticID Defend = StrategyPacket_TacticID_Defend;
  static const TacticID Attack = StrategyPacket_TacticID_Attack;
  static const TacticID Steal = StrategyPacket_TacticID_Steal;
  static const TacticID Velocity = StrategyPacket_TacticID_Velocity;
  static inline bool TacticID_IsValid(int value) {
    return StrategyPacket_TacticID_IsValid(value);
  }
  static const TacticID TacticID_MIN =
    StrategyPacket_TacticID_TacticID_MIN;
  static const TacticID TacticID_MAX =
    StrategyPacket_TacticID_TacticID_MAX;
  static const int TacticID_ARRAYSIZE =
    StrategyPacket_TacticID_TacticID_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  TacticID_descriptor() {
    return StrategyPacket_TacticID_descriptor();
  }
  static inline const ::std::string& TacticID_Name(TacticID value) {
    return StrategyPacket_TacticID_Name(value);
  }
  static inline bool TacticID_Parse(const ::std::string& name,
      TacticID* value) {
    return StrategyPacket_TacticID_Parse(name, value);
  }
  
  typedef StrategyPacket_Which Which;
  static const Which PLAY = StrategyPacket_Which_PLAY;
  static const Which TACTIC = StrategyPacket_Which_TACTIC;
  static inline bool Which_IsValid(int value) {
    return StrategyPacket_Which_IsValid(value);
  }
  static const Which Which_MIN =
    StrategyPacket_Which_Which_MIN;
  static const Which Which_MAX =
    StrategyPacket_Which_Which_MAX;
  static const int Which_ARRAYSIZE =
    StrategyPacket_Which_Which_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Which_descriptor() {
    return StrategyPacket_Which_descriptor();
  }
  static inline const ::std::string& Which_Name(Which value) {
    return StrategyPacket_Which_Name(value);
  }
  static inline bool Which_Parse(const ::std::string& name,
      Which* value) {
    return StrategyPacket_Which_Parse(name, value);
  }
  
  // accessors -------------------------------------------------------
  
  // optional .StrategyPacket.Play play = 1;
  inline bool has_play() const;
  inline void clear_play();
  static const int kPlayFieldNumber = 1;
  inline ::StrategyPacket_Play play() const;
  inline void set_play(::StrategyPacket_Play value);
  
  // optional .StrategyPacket.Tactic tactic = 2;
  inline bool has_tactic() const;
  inline void clear_tactic();
  static const int kTacticFieldNumber = 2;
  inline const ::StrategyPacket_Tactic& tactic() const;
  inline ::StrategyPacket_Tactic* mutable_tactic();
  inline ::StrategyPacket_Tactic* release_tactic();
  
  // required .StrategyPacket.Which which = 3;
  inline bool has_which() const;
  inline void clear_which();
  static const int kWhichFieldNumber = 3;
  inline ::StrategyPacket_Which which() const;
  inline void set_which(::StrategyPacket_Which value);
  
  // @@protoc_insertion_point(class_scope:StrategyPacket)
 private:
  inline void set_has_play();
  inline void clear_has_play();
  inline void set_has_tactic();
  inline void clear_has_tactic();
  inline void set_has_which();
  inline void clear_has_which();
  
  ::google::protobuf::UnknownFieldSet _unknown_fields_;
  
  ::StrategyPacket_Tactic* tactic_;
  int play_;
  int which_;
  
  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];
  
  friend void  protobuf_AddDesc_command_2dpacket_2eproto();
  friend void protobuf_AssignDesc_command_2dpacket_2eproto();
  friend void protobuf_ShutdownFile_command_2dpacket_2eproto();
  
  void InitAsDefaultInstance();
  static StrategyPacket* default_instance_;
};
// ===================================================================


// ===================================================================

// StrategyPacket_Tactic

// required .StrategyPacket.TacticID tID = 1;
inline bool StrategyPacket_Tactic::has_tid() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void StrategyPacket_Tactic::set_has_tid() {
  _has_bits_[0] |= 0x00000001u;
}
inline void StrategyPacket_Tactic::clear_has_tid() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void StrategyPacket_Tactic::clear_tid() {
  tid_ = 0;
  clear_has_tid();
}
inline ::StrategyPacket_TacticID StrategyPacket_Tactic::tid() const {
  return static_cast< ::StrategyPacket_TacticID >(tid_);
}
inline void StrategyPacket_Tactic::set_tid(::StrategyPacket_TacticID value) {
  GOOGLE_DCHECK(::StrategyPacket_TacticID_IsValid(value));
  set_has_tid();
  tid_ = value;
}

// required int32 botID = 2;
inline bool StrategyPacket_Tactic::has_botid() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void StrategyPacket_Tactic::set_has_botid() {
  _has_bits_[0] |= 0x00000002u;
}
inline void StrategyPacket_Tactic::clear_has_botid() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void StrategyPacket_Tactic::clear_botid() {
  botid_ = 0;
  clear_has_botid();
}
inline ::google::protobuf::int32 StrategyPacket_Tactic::botid() const {
  return botid_;
}
inline void StrategyPacket_Tactic::set_botid(::google::protobuf::int32 value) {
  set_has_botid();
  botid_ = value;
}

// -------------------------------------------------------------------

// StrategyPacket

// optional .StrategyPacket.Play play = 1;
inline bool StrategyPacket::has_play() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void StrategyPacket::set_has_play() {
  _has_bits_[0] |= 0x00000001u;
}
inline void StrategyPacket::clear_has_play() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void StrategyPacket::clear_play() {
  play_ = 0;
  clear_has_play();
}
inline ::StrategyPacket_Play StrategyPacket::play() const {
  return static_cast< ::StrategyPacket_Play >(play_);
}
inline void StrategyPacket::set_play(::StrategyPacket_Play value) {
  GOOGLE_DCHECK(::StrategyPacket_Play_IsValid(value));
  set_has_play();
  play_ = value;
}

// optional .StrategyPacket.Tactic tactic = 2;
inline bool StrategyPacket::has_tactic() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void StrategyPacket::set_has_tactic() {
  _has_bits_[0] |= 0x00000002u;
}
inline void StrategyPacket::clear_has_tactic() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void StrategyPacket::clear_tactic() {
  if (tactic_ != NULL) tactic_->::StrategyPacket_Tactic::Clear();
  clear_has_tactic();
}
inline const ::StrategyPacket_Tactic& StrategyPacket::tactic() const {
  return tactic_ != NULL ? *tactic_ : *default_instance_->tactic_;
}
inline ::StrategyPacket_Tactic* StrategyPacket::mutable_tactic() {
  set_has_tactic();
  if (tactic_ == NULL) tactic_ = new ::StrategyPacket_Tactic;
  return tactic_;
}
inline ::StrategyPacket_Tactic* StrategyPacket::release_tactic() {
  clear_has_tactic();
  ::StrategyPacket_Tactic* temp = tactic_;
  tactic_ = NULL;
  return temp;
}

// required .StrategyPacket.Which which = 3;
inline bool StrategyPacket::has_which() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void StrategyPacket::set_has_which() {
  _has_bits_[0] |= 0x00000004u;
}
inline void StrategyPacket::clear_has_which() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void StrategyPacket::clear_which() {
  which_ = 1;
  clear_has_which();
}
inline ::StrategyPacket_Which StrategyPacket::which() const {
  return static_cast< ::StrategyPacket_Which >(which_);
}
inline void StrategyPacket::set_which(::StrategyPacket_Which value) {
  GOOGLE_DCHECK(::StrategyPacket_Which_IsValid(value));
  set_has_which();
  which_ = value;
}


// @@protoc_insertion_point(namespace_scope)

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::StrategyPacket_Play>() {
  return ::StrategyPacket_Play_descriptor();
}
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::StrategyPacket_TacticID>() {
  return ::StrategyPacket_TacticID_descriptor();
}
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::StrategyPacket_Which>() {
  return ::StrategyPacket_Which_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_command_2dpacket_2eproto__INCLUDED
