/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2021, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file     Unit.h
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#ifndef __UNIT__
#define __UNIT__

#include "CommonDef.h"
#include "Common.h"
#include "Mv.h"
#include "MotionInfo.h"
#include "ChromaFormat.h"

#ifdef STANDALONE_ENTROPY_CODEC
#include "unit.hpp"
#endif


// ---------------------------------------------------------------------------
// tools
// ---------------------------------------------------------------------------
struct PLTBuf {
  uint8_t        curPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel            curPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE];

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::PLTBuf() const
  {
    EntropyCoding::PLTBuf result;
    EntropyCoding::copy_array(curPLTSize, result.curPLTSize);
    for (int i = 0; i < result.curPLT.size(); ++i)
    {
      EntropyCoding::copy_array(curPLT[i], result.curPLT[i]);
    }

    return result;
  }

  const PLTBuf &operator=(const EntropyCoding::PLTBuf &rhs)
  {
    std::copy(rhs.curPLTSize.begin(), rhs.curPLTSize.end(), curPLTSize);
    for (int i = 0; i < MAX_NUM_COMPONENT; ++i)
    {
      std::copy(rhs.curPLT[i].begin(), rhs.curPLT[i].end(), curPLT[i]);
    }
    return *this;
  }
#endif
};
inline Position recalcPosition(const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Position &pos)
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return pos;
  }
  else if (isLuma(srcCId) && isChroma(dstCId))
  {
    return Position(pos.x >> getComponentScaleX(dstCId, _cf), pos.y >> getComponentScaleY(dstCId, _cf));
  }
  else
  {
    return Position(pos.x << getComponentScaleX(srcCId, _cf), pos.y << getComponentScaleY(srcCId, _cf));
  }
}

inline Position recalcPosition( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Position &pos )
{
  if( srcCHt == dstCHt )
  {
    return pos;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Position( pos.x >> getChannelTypeScaleX( dstCHt, _cf ), pos.y >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Position( pos.x << getChannelTypeScaleX( srcCHt, _cf ), pos.y << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ComponentID srcCId, const ComponentID dstCId, const Size &size )
{
  if( toChannelType( srcCId ) == toChannelType( dstCId ) )
  {
    return size;
  }
  else if( isLuma( srcCId ) && isChroma( dstCId ) )
  {
    return Size( size.width >> getComponentScaleX( dstCId, _cf ), size.height >> getComponentScaleY( dstCId, _cf ) );
  }
  else
  {
    return Size( size.width << getComponentScaleX( srcCId, _cf ), size.height << getComponentScaleY( srcCId, _cf ) );
  }
}

inline Size recalcSize( const ChromaFormat _cf, const ChannelType srcCHt, const ChannelType dstCHt, const Size &size )
{
  if( srcCHt == dstCHt )
  {
    return size;
  }
  else if( isLuma( srcCHt ) && isChroma( dstCHt ) )
  {
    return Size( size.width >> getChannelTypeScaleX( dstCHt, _cf ), size.height >> getChannelTypeScaleY( dstCHt, _cf ) );
  }
  else
  {
    return Size( size.width << getChannelTypeScaleX( srcCHt, _cf ), size.height << getChannelTypeScaleY( srcCHt, _cf ) );
  }
}

// ---------------------------------------------------------------------------
// block definition
// ---------------------------------------------------------------------------

struct CompArea : public Area
{
  CompArea() : Area(), chromaFormat(NUM_CHROMA_FORMAT), compID(MAX_NUM_TBLOCKS)                                                                                                                                 { }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const Area &_area, const bool isLuma = false)                                          : Area(_area),          chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const Position& _pos, const Size& _size, const bool isLuma = false)                    : Area(_pos, _size),    chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }
  CompArea(const ComponentID _compID, const ChromaFormat _cf, const uint32_t _x, const uint32_t _y, const uint32_t _w, const uint32_t _h, const bool isLuma = false) : Area(_x, _y, _w, _h), chromaFormat(_cf), compID(_compID) { if (isLuma) xRecalcLumaToChroma(); }

  ChromaFormat chromaFormat;
  ComponentID compID;

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::CompArea() const
  {
    return EntropyCoding::CompArea(static_cast<EntropyCoding::ComponentID>(compID),
                                   static_cast<EntropyCoding::ChromaFormat>(chromaFormat), x, y, width, height);
  }

  const CompArea &operator=(const EntropyCoding::CompArea &rhs)
  {
    chromaFormat = static_cast<ChromaFormat>(rhs.chromaFormat);
    compID       = static_cast<ComponentID>(rhs.compID);
    x            = rhs.x;
    y            = rhs.y;
    width        = rhs.width;
    height       = rhs.height;
    return *this;
  }
#endif

  Position chromaPos() const;
  Position lumaPos()   const;

  Size     chromaSize() const;
  Size     lumaSize()   const;

  Position compPos( const ComponentID compID ) const;
  Position chanPos( const ChannelType chType ) const;

  Position topLeftComp    (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, *this);                                                     }
  Position topRightComp   (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), y                          }); }
  Position bottomLeftComp (const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { x                        , (PosType) (y + height - 1 )}); }
  Position bottomRightComp(const ComponentID _compID) const { return recalcPosition(chromaFormat, compID, _compID, { (PosType) (x + width - 1), (PosType) (y + height - 1 )}); }

  bool valid() const { return chromaFormat < NUM_CHROMA_FORMAT && compID < MAX_NUM_TBLOCKS && width != 0 && height != 0; }

  const bool operator==(const CompArea &other) const
  {
    if (chromaFormat != other.chromaFormat) return false;
    if (compID       != other.compID)       return false;

    return Position::operator==(other) && Size::operator==(other);
  }

  const bool operator!=(const CompArea &other) const { return !(operator==(other)); }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  void     resizeTo          (const Size& newSize)          { Size::resizeTo(newSize); }
#endif
  void     repositionTo      (const Position& newPos)       { Position::repositionTo(newPos); }
  void     positionRelativeTo(const CompArea& origCompArea) { Position::relativeTo(origCompArea); }

private:

  void xRecalcLumaToChroma();
};

inline CompArea clipArea(const CompArea &compArea, const Area &boundingBox)
{
  return CompArea(compArea.compID, compArea.chromaFormat, clipArea((const Area&) compArea, boundingBox));
}

// ---------------------------------------------------------------------------
// unit definition
// ---------------------------------------------------------------------------

typedef static_vector<CompArea, MAX_NUM_TBLOCKS> UnitBlocksType;

struct UnitArea
{
  ChromaFormat chromaFormat;
  UnitBlocksType blocks;

  UnitArea() : chromaFormat(NUM_CHROMA_FORMAT) { }
  UnitArea(const ChromaFormat _chromaFormat);
  UnitArea(const ChromaFormat _chromaFormat, const Area &area);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea  &blkY);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY);
  UnitArea(const ChromaFormat _chromaFormat, const CompArea  &blkY, const CompArea  &blkCb, const CompArea  &blkCr);
  UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,       CompArea &&blkCb,       CompArea &&blkCr);

#ifdef STANDALONE_ENTROPY_CODEC
  UnitArea(const EntropyCoding::UnitArea &rhs)
    : chromaFormat(static_cast<ChromaFormat>(rhs.chromaFormat)), blocks(rhs.blocks.begin(), rhs.blocks.end())
  {
  }

  UnitArea &operator=(const EntropyCoding::UnitArea &rhs)
  {
    chromaFormat = static_cast<ChromaFormat>(rhs.chromaFormat);
    for (int i = 0; i < blocks.size(); ++i)
    {
      blocks[i] = rhs.blocks[i];
    }
    return *this;
  }

  operator EntropyCoding::UnitArea() const
  {
    return EntropyCoding::UnitArea(static_cast<EntropyCoding::ChromaFormat>(chromaFormat), blocks);
  }
#endif

  CompArea &      Y() { return blocks[COMPONENT_Y]; }
  const CompArea& Y()                            const { return blocks[COMPONENT_Y];  }
        CompArea& Cb()                                 { return blocks[COMPONENT_Cb]; }
  const CompArea& Cb()                           const { return blocks[COMPONENT_Cb]; }
        CompArea& Cr()                                 { return blocks[COMPONENT_Cr]; }
  const CompArea& Cr()                           const { return blocks[COMPONENT_Cr]; }

        CompArea& block(const ComponentID comp)       { return blocks[comp]; }
  const CompArea& block(const ComponentID comp) const { return blocks[comp]; }

  bool contains(const UnitArea& other) const;
  bool contains(const UnitArea& other, const ChannelType chType) const;

        CompArea& operator[]( const int n )       { return blocks[n]; }
  const CompArea& operator[]( const int n ) const { return blocks[n]; }

  const bool operator==(const UnitArea &other) const
  {
    if (chromaFormat != other.chromaFormat)   return false;
    if (blocks.size() != other.blocks.size()) return false;

    for (uint32_t i = 0; i < blocks.size(); i++)
    {
      if (blocks[i] != other.blocks[i]) return false;
    }

    return true;
  }

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS
  void resizeTo    (const UnitArea& unit);
#endif
  void repositionTo(const UnitArea& unit);

  const bool operator!=(const UnitArea &other) const { return !(*this == other); }

  const Position& lumaPos () const { return Y(); }
  const Size&     lumaSize() const { return Y(); }

  const Position& chromaPos () const { return Cb(); }
  const Size&     chromaSize() const { return Cb(); }

  const UnitArea  singleComp(const ComponentID compID) const;
  const UnitArea  singleChan(const ChannelType chType) const;

  const SizeType  lwidth()  const { return Y().width; }  /*! luma width  */
  const SizeType  lheight() const { return Y().height; } /*! luma height */

  const PosType   lx() const { return Y().x; }           /*! luma x-pos */
  const PosType   ly() const { return Y().y; }           /*! luma y-pos */

  bool valid() const { return chromaFormat != NUM_CHROMA_FORMAT && blocks.size() > 0; }
};

inline UnitArea clipArea(const UnitArea &area, const UnitArea &boundingBox)
{
  UnitArea ret(area.chromaFormat);

  for (uint32_t i = 0; i < area.blocks.size(); i++)
  {
    ret.blocks.push_back(clipArea(area.blocks[i], boundingBox.blocks[i]));
  }

  return ret;
}

struct UnitAreaRelative : public UnitArea
{
  UnitAreaRelative(const UnitArea& origUnit, const UnitArea& unit)
  {
    *((UnitArea*)this) = unit;
    for(uint32_t i = 0; i < blocks.size(); i++)
    {
      blocks[i].positionRelativeTo(origUnit.blocks[i]);
    }
  }
};

class SPS;
class VPS;
class DCI;
class PPS;
class Slice;

// ---------------------------------------------------------------------------
// coding unit
// ---------------------------------------------------------------------------

#include "Buffer.h"

struct TransformUnit;
struct PredictionUnit;
class  CodingStructure;

struct CodingUnit : public UnitArea
{
  CodingStructure *cs;
  Slice *slice;
  ChannelType    chType;

  PredMode       predMode;

  uint8_t          depth;   // number of all splits, applied with generalized splits
  uint8_t          qtDepth; // number of applied quad-splits, before switching to the multi-type-tree (mtt)
  // a triple split would increase the mtDepth by 1, but the qtDepth by 2 in the first and last part and by 1 in the middle part (because of the 1-2-1 split proportions)
  uint8_t          btDepth; // number of applied binary splits, after switching to the mtt (or it's equivalent)
  uint8_t          mtDepth; // the actual number of splits after switching to mtt (equals btDepth if only binary splits are allowed)
  int8_t          chromaQpAdj;
  int8_t          qp;
  SplitSeries    splitSeries;
  TreeType       treeType;
  ModeType       modeType;
  ModeTypeSeries modeTypeSeries;
  bool           skip;
  bool           mmvdSkip;
  bool           affine;
  int            affineType;
  bool           colorTransform;
  bool           geoFlag;
  int            bdpcmMode;
  int            bdpcmModeChroma;
  uint8_t          imv;
  bool           rootCbf;
  uint8_t        sbtInfo;
  uint32_t           tileIdx;
  uint8_t         mtsFlag;
  uint32_t        lfnstIdx;
  uint8_t         BcwIdx;
  int             refIdxBi[2];
  bool           mipFlag;

  // needed for fast imv mode decisions
  int8_t          imvNumCand;
  uint8_t          smvdMode;
  uint8_t        ispMode;
  bool           useEscape[MAX_NUM_CHANNEL_TYPE];
  bool           useRotation[MAX_NUM_CHANNEL_TYPE];
  bool           reuseflag[MAX_NUM_CHANNEL_TYPE][MAXPLTPREDSIZE];
  uint8_t        lastPLTSize[MAX_NUM_CHANNEL_TYPE];
  uint8_t        reusePLTSize[MAX_NUM_CHANNEL_TYPE];
  uint8_t        curPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel            curPLT[MAX_NUM_COMPONENT][MAXPLTSIZE];

  CodingUnit() : chType( CH_L ) { }
  CodingUnit(const UnitArea &unit);
  CodingUnit(const ChromaFormat _chromaFormat, const Area &area);

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::CodingUnit() const
  {
    return EntropyCoding::CodingUnit(static_cast<UnitArea>(*this), static_cast<EntropyCoding::ChannelType>(chType),
                                     static_cast<EntropyCoding::PredMode>(predMode), depth, qtDepth, btDepth, mtDepth,
                                     chromaQpAdj, qp, splitSeries, static_cast<EntropyCoding::TreeType>(treeType),
                                     static_cast<EntropyCoding::ModeType>(modeType), modeTypeSeries, skip, mmvdSkip,
                                     affine, affineType, colorTransform, geoFlag, bdpcmMode, bdpcmModeChroma, imv,
                                     rootCbf, sbtInfo, tileIdx, lfnstIdx, BcwIdx, mipFlag, smvdMode, ispMode, useEscape,
                                     useRotation, reuseflag, lastPLTSize, reusePLTSize, curPLTSize, curPLT, idx);
  }

  const CodingUnit &operator=(const EntropyCoding::CodingUnit &rhs)
  {
    chromaFormat    = static_cast<ChromaFormat>(rhs.chromaFormat);
    blocks          = rhs.blocks;
    chType          = static_cast<ChannelType>(rhs.chType);
    predMode        = static_cast<PredMode>(rhs.predMode);
    depth           = rhs.depth;
    qtDepth         = rhs.qtDepth;
    btDepth         = rhs.btDepth;
    mtDepth         = rhs.mtDepth;
    chromaQpAdj     = rhs.chromaQpAdj;
    qp              = rhs.qp;
    splitSeries     = rhs.splitSeries;
    treeType        = static_cast<TreeType>(rhs.treeType);
    modeType        = static_cast<ModeType>(rhs.modeType);
    modeTypeSeries  = rhs.modeTypeSeries;
    skip            = rhs.skip;
    mmvdSkip        = rhs.mmvdSkip;
    affine          = rhs.affine;
    affineType      = rhs.affineType;
    colorTransform  = rhs.colorTransform;
    geoFlag         = rhs.geoFlag;
    bdpcmMode       = rhs.bdpcmMode;
    bdpcmModeChroma = rhs.bdpcmModeChroma;
    imv             = rhs.imv;
    rootCbf         = rhs.rootCbf;
    sbtInfo         = rhs.sbtInfo;
    tileIdx         = rhs.tileIdx;
    lfnstIdx        = rhs.lfnstIdx;
    BcwIdx          = rhs.BcwIdx;
    mipFlag         = rhs.mipFlag;
    smvdMode        = rhs.smvdMode;
    ispMode         = rhs.ispMode;
    idx             = rhs.idx;

    std::copy(rhs.useEscape.begin(), rhs.useEscape.end(), useEscape);
    std::copy(rhs.useRotation.begin(), rhs.useRotation.end(), useRotation);
    for (int i = 0; i < MAX_NUM_CHANNEL_TYPE; ++i)
    {
      std::copy(rhs.reuseflag[i].begin(), rhs.reuseflag[i].end(), reuseflag[i]);
    }
    std::copy(rhs.lastPLTSize.begin(), rhs.lastPLTSize.end(), lastPLTSize);
    std::copy(rhs.reusePLTSize.begin(), rhs.reusePLTSize.end(), reusePLTSize);
    std::copy(rhs.curPLTSize.begin(), rhs.curPLTSize.end(), curPLTSize);
    for (int i = 0; i < MAX_NUM_COMPONENT; ++i)
    {
      std::copy(rhs.curPLT[i].begin(), rhs.curPLT[i].end(), curPLT[i]);
    }
    return *this;
  }
#endif

  CodingUnit& operator=( const CodingUnit& other );

  void initData();

  unsigned    idx;
  CodingUnit *next;

  PredictionUnit *firstPU;
  PredictionUnit *lastPU;

  TransformUnit *firstTU;
  TransformUnit *lastTU;

  const uint8_t     getSbtIdx() const { assert( ( ( sbtInfo >> 0 ) & 0xf ) < NUMBER_SBT_IDX ); return ( sbtInfo >> 0 ) & 0xf; }
  const uint8_t     getSbtPos() const { return ( sbtInfo >> 4 ) & 0x3; }
  void              setSbtIdx( uint8_t idx ) { CHECK( idx >= NUMBER_SBT_IDX, "sbt_idx wrong" ); sbtInfo = ( idx << 0 ) + ( sbtInfo & 0xf0 ); }
  void              setSbtPos( uint8_t pos ) { CHECK( pos >= 4, "sbt_pos wrong" ); sbtInfo = ( pos << 4 ) + ( sbtInfo & 0xcf ); }
  uint8_t           getSbtTuSplit() const;
  const uint8_t     checkAllowedSbt() const;
  const bool        checkCCLMAllowed() const;
  const bool        isSepTree() const;
  const bool        isLocalSepTree() const;
  const bool        isConsInter() const { return modeType == MODE_TYPE_INTER; }
  const bool        isConsIntra() const { return modeType == MODE_TYPE_INTRA; }
};

// ---------------------------------------------------------------------------
// prediction unit
// ---------------------------------------------------------------------------

struct IntraPredictionData
{
  uint32_t intraDir[MAX_NUM_CHANNEL_TYPE];
  bool     mipTransposedFlag;
  int      multiRefIdx;

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::IntraPredictionData() const
  {
    return EntropyCoding::IntraPredictionData(intraDir, mipTransposedFlag, multiRefIdx);
  }
#endif
};

struct InterPredictionData
{
  bool      mergeFlag;
  bool      regularMergeFlag;
  uint8_t     mergeIdx;
  uint8_t     geoSplitDir;
  uint8_t     geoMergeIdx0;
  uint8_t     geoMergeIdx1;
  bool           mmvdMergeFlag;
  uint32_t       mmvdMergeIdx;
  uint8_t     interDir;
  uint8_t     mvpIdx  [NUM_REF_PIC_LIST_01];
  uint8_t     mvpNum  [NUM_REF_PIC_LIST_01];
  Mv        mvd     [NUM_REF_PIC_LIST_01];
  Mv        mv      [NUM_REF_PIC_LIST_01];
#if GDR_ENABLED 
  bool      mvSolid[NUM_REF_PIC_LIST_01];
  bool      mvValid[NUM_REF_PIC_LIST_01];
  bool      mvpSolid[NUM_REF_PIC_LIST_01];
  MvpType   mvpType[NUM_REF_PIC_LIST_01];
  Position  mvpPos[NUM_REF_PIC_LIST_01];
#endif
  int16_t     refIdx  [NUM_REF_PIC_LIST_01];
  MergeType mergeType;
  bool      mvRefine;
  Mv        mvdL0SubPu[MAX_NUM_SUBCU_DMVR];
  Mv        mvdAffi [NUM_REF_PIC_LIST_01][3];
  Mv        mvAffi[NUM_REF_PIC_LIST_01][3];
#if GDR_ENABLED
  bool      mvAffiSolid[NUM_REF_PIC_LIST_01][3];
  bool      mvAffiValid[NUM_REF_PIC_LIST_01][3];
  MvpType   mvAffiType[NUM_REF_PIC_LIST_01][3];
  Position  mvAffiPos[NUM_REF_PIC_LIST_01][3];
#endif
  bool      ciipFlag;

  Mv        bv;                             // block vector for IBC
  Mv        bvd;                            // block vector difference for IBC
  uint8_t   mmvdEncOptMode;                  // 0: no action 1: skip chroma MC for MMVD candidate pre-selection 2: skip chroma MC and BIO for MMVD candidate pre-selection

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::InterPredictionData() const
  {
    std::array<EntropyCoding::Mv, NUM_REF_PIC_LIST_01> _mvd;
    EntropyCoding::copy_array(mvd, _mvd);

    std::array<EntropyCoding::Mv, NUM_REF_PIC_LIST_01> _mv;
    EntropyCoding::copy_array(mv, _mv);

    EntropyCoding::MvdAffi _mvdAffi;
    for (int i = 0; i < _mvdAffi.size(); ++i)
    {
      EntropyCoding::copy_array(mvdAffi[i], _mvdAffi[i]);
    }

    return EntropyCoding::InterPredictionData(
      mergeFlag, regularMergeFlag, mergeIdx, geoSplitDir, geoMergeIdx0, geoMergeIdx1, mmvdMergeFlag, mmvdMergeIdx,
      interDir, mvpIdx, std::move(_mvd), std::move(_mv), refIdx, static_cast<EntropyCoding::MergeType>(mergeType),
      std::move(_mvdAffi), ciipFlag);
  }
#endif
};

struct PredictionUnit : public UnitArea, public IntraPredictionData, public InterPredictionData
{
  CodingUnit      *cu;
  CodingStructure *cs;
  ChannelType      chType;

  // constructors
  PredictionUnit(): chType( CH_L ) { }
  PredictionUnit(const UnitArea &unit);
  PredictionUnit(const ChromaFormat _chromaFormat, const Area &area);

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::PredictionUnit() const
  {
    return EntropyCoding::PredictionUnit(static_cast<UnitArea>(*this), static_cast<IntraPredictionData>(*this),
                                         static_cast<InterPredictionData>(*this),
                                         static_cast<EntropyCoding::ChannelType>(chType), idx);
  }

  const PredictionUnit &operator=(const EntropyCoding::PredictionUnit &rhs)
  {
    chromaFormat = static_cast<ChromaFormat>(rhs.chromaFormat);
    blocks       = rhs.blocks;

    std::copy(rhs.intraDir.begin(), rhs.intraDir.end(), intraDir);
    mipTransposedFlag = rhs.mipTransposedFlag;
    multiRefIdx       = rhs.multiRefIdx;

    mergeFlag        = rhs.mergeFlag;
    regularMergeFlag = rhs.regularMergeFlag;
    mergeIdx         = rhs.mergeIdx;
    geoSplitDir      = rhs.geoSplitDir;
    geoMergeIdx0     = rhs.geoMergeIdx0;
    geoMergeIdx1     = rhs.geoMergeIdx1;
    mmvdMergeFlag    = rhs.mmvdMergeFlag;
    mmvdMergeIdx     = rhs.mmvdMergeIdx;
    interDir         = rhs.interDir;
    mergeType        = static_cast<MergeType>(rhs.mergeType);
    ciipFlag         = rhs.ciipFlag;
    std::copy(rhs.mvpIdx.begin(), rhs.mvpIdx.end(), mvpIdx);
    std::copy(rhs.mvd.begin(), rhs.mvd.end(), mvd);
    std::copy(rhs.mv.begin(), rhs.mv.end(), mv);
    std::copy(rhs.refIdx.begin(), rhs.refIdx.end(), refIdx);
    for (int i = 0; i < NUM_REF_PIC_LIST_01; ++i)
    {
      std::copy(rhs.mvdAffi[i].begin(), rhs.mvdAffi[i].end(), mvdAffi[i]);
    }

    chType = static_cast<ChannelType>(rhs.chType);
    idx    = rhs.idx;
    return *this;
  }
#endif

  void initData();

  PredictionUnit& operator=(const IntraPredictionData& predData);
  PredictionUnit& operator=(const InterPredictionData& predData);
  PredictionUnit& operator=(const PredictionUnit& other);
  PredictionUnit& operator=(const MotionInfo& mi);

  unsigned        idx;

  PredictionUnit *next;

  // for accessing motion information, which can have higher resolution than PUs (should always be used, when accessing neighboring motion information)
  const MotionInfo& getMotionInfo() const;
  const MotionInfo& getMotionInfo( const Position& pos ) const;
  MotionBuf         getMotionBuf();
  CMotionBuf        getMotionBuf() const;
};

// ---------------------------------------------------------------------------
// transform unit
// ---------------------------------------------------------------------------

struct TransformUnit : public UnitArea
{
  CodingUnit      *cu;
  CodingStructure *cs;
  ChannelType      chType;
  int              m_chromaResScaleInv;

  uint8_t        depth;
  uint8_t        mtsIdx     [ MAX_NUM_TBLOCKS ];
  bool           noResidual;
  uint8_t        jointCbCr;
  uint8_t        cbf        [ MAX_NUM_TBLOCKS ];

  TransformUnit() : chType( CH_L ) { }
  TransformUnit(const UnitArea& unit);
  TransformUnit(const ChromaFormat _chromaFormat, const Area &area);

#ifdef STANDALONE_ENTROPY_CODEC
  operator EntropyCoding::TransformUnit() const
  {
    return EntropyCoding::TransformUnit(static_cast<UnitArea>(*this), static_cast<EntropyCoding::ChannelType>(chType),
                                        depth, mtsIdx, noResidual, jointCbCr, cbf, idx, m_coeffs, m_pcmbuf, m_runType);
  }

  const TransformUnit &operator=(EntropyCoding::TransformUnit &rhs)
  {
    chromaFormat = static_cast<ChromaFormat>(rhs.chromaFormat);
    blocks       = rhs.blocks;

    chType     = static_cast<ChannelType>(rhs.chType);
    depth      = rhs.depth;
    noResidual = rhs.noResidual;
    jointCbCr  = rhs.jointCbCr;
    idx        = rhs.idx;

    std::copy(rhs.mtsIdx.begin(), rhs.mtsIdx.end(), mtsIdx);
    std::copy(rhs.cbf.begin(), rhs.cbf.end(), cbf);
    for (int i = 0; i < MAX_NUM_TBLOCKS; ++i)
    {
      m_coeffs[i] = rhs.getCoeffs()[i];
      m_pcmbuf[i] = rhs.getPcmBuf()[i];
      if (i < MAX_NUM_TBLOCKS - 1)
      {
        m_runType[i] = rhs.getRunType()[i];
      }
    }
    return *this;
  }
#endif

  void initData();

  unsigned       idx;
  TransformUnit *next;
  TransformUnit *prev;
  void init(TCoeff **coeffs, Pel **pcmbuf, bool **runType);

  TransformUnit& operator=(const TransformUnit& other);
  void copyComponentFrom  (const TransformUnit& other, const ComponentID compID);
  void checkTuNoResidual( unsigned idx );
  int  getTbAreaAfterCoefZeroOut(ComponentID compID) const;

         CoeffBuf getCoeffs(const ComponentID id);
  const CCoeffBuf getCoeffs(const ComponentID id) const;
         PelBuf   getPcmbuf(const ComponentID id);
  const CPelBuf   getPcmbuf(const ComponentID id) const;
        int       getChromaAdj( )                 const;
        void      setChromaAdj(int i);
         PelBuf   getcurPLTIdx(const ComponentID id);
  const CPelBuf   getcurPLTIdx(const ComponentID id) const;
         PLTtypeBuf   getrunType(const ComponentID id);
  const CPLTtypeBuf   getrunType(const ComponentID id) const;
         PLTescapeBuf getescapeValue(const ComponentID id);
  const CPLTescapeBuf getescapeValue(const ComponentID id) const;
        Pel*      getPLTIndex(const ComponentID id);
        bool*     getRunTypes(const ComponentID id);

private:
  TCoeff *m_coeffs[ MAX_NUM_TBLOCKS ];
  Pel    *m_pcmbuf[ MAX_NUM_TBLOCKS ];
  bool   *m_runType[ MAX_NUM_TBLOCKS - 1 ];
};

// ---------------------------------------------------------------------------
// Utility class for easy for-each like unit traversing
// ---------------------------------------------------------------------------

#include <iterator>

template<typename T>
class UnitIterator : public std::iterator<std::forward_iterator_tag, T>
{
private:
  T* m_punit;

public:
  UnitIterator(           ) : m_punit( nullptr ) { }
  UnitIterator( T* _punit ) : m_punit( _punit  ) { }

  typedef T&       reference;
  typedef T const& const_reference;
  typedef T*       pointer;
  typedef T const* const_pointer;

  reference        operator*()                                      { return *m_punit; }
  const_reference  operator*()                                const { return *m_punit; }
  pointer          operator->()                                     { return  m_punit; }
  const_pointer    operator->()                               const { return  m_punit; }

  UnitIterator<T>& operator++()                                     { m_punit = m_punit->next; return *this; }
  UnitIterator<T>  operator++( int )                                { auto x = *this; ++( *this ); return x; }
  bool             operator!=( const UnitIterator<T>& other ) const { return m_punit != other.m_punit; }
  bool             operator==( const UnitIterator<T>& other ) const { return m_punit == other.m_punit; }
};

template<typename T>
class UnitTraverser
{
private:
  T* m_begin;
  T* m_end;

public:
  UnitTraverser(                    ) : m_begin( nullptr ), m_end( nullptr ) { }
  UnitTraverser( T* _begin, T* _end ) : m_begin( _begin  ), m_end( _end    ) { }

  typedef T                     value_type;
  typedef size_t                size_type;
  typedef T&                    reference;
  typedef T const&              const_reference;
  typedef T*                    pointer;
  typedef T const*              const_pointer;
  typedef UnitIterator<T>       iterator;
  typedef UnitIterator<const T> const_iterator;

  iterator        begin()        { return UnitIterator<T>( m_begin ); }
  const_iterator  begin()  const { return UnitIterator<T>( m_begin ); }
  const_iterator  cbegin() const { return UnitIterator<T>( m_begin ); }
  iterator        end()          { return UnitIterator<T>( m_end   ); }
  const_iterator  end()    const { return UnitIterator<T>( m_end   ); }
  const_iterator  cend()   const { return UnitIterator<T>( m_end   ); }
};

typedef UnitTraverser<CodingUnit>     CUTraverser;
typedef UnitTraverser<PredictionUnit> PUTraverser;
typedef UnitTraverser<TransformUnit>  TUTraverser;

typedef UnitTraverser<const CodingUnit>     cCUTraverser;
typedef UnitTraverser<const PredictionUnit> cPUTraverser;
typedef UnitTraverser<const TransformUnit>  cTUTraverser;

#endif

