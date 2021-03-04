//=============================================================================================================
/**
 * @file     eventgroup.cpp
 * @author   Gabriel Motta <gbmotta@mgh.harvard.edu>;
 *           Juan Garcia-Prieto <juangpc@gmail.com>
 * @since    0.1.8
 * @date     February, 2021
 *
 * @section  LICENSE
 *
 * Copyright (C) 2021, Gabriel Motta, Juan Garcia-Prieto. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 * the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *       following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *       the following disclaimer in the documentation and/or other materials provided with the distribution.
 *     * Neither the name of MNE-CPP authors nor the names of its contributors may be used
 *       to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @brief     EventGroup definition.
 *
 */

//=============================================================================================================
// INCLUDES
//=============================================================================================================

#include "eventgroup.h"
#include "event.h"

#include <stdlib.h>
#include <ctime>

//=============================================================================================================
// QT INCLUDES
//=============================================================================================================

//=============================================================================================================
// EIGEN INCLUDES
//=============================================================================================================

//=============================================================================================================
// USED NAMESPACES
//=============================================================================================================

using namespace EVENTSINTERNAL;

//=============================================================================================================
// INIT STATIC MEMBERS
//=============================================================================================================

constexpr static const unsigned char defaultGroupColor[] = { 0xC0, 0xFF, 0xEE };
constexpr static const unsigned char defaultGroupTransparency = 0xFF;

//=============================================================================================================
// DEFINE MEMBER METHODS
//=============================================================================================================

EVENTSLIB::RgbColor::RgbColor()
: RgbColor(defaultGroupColor[0], defaultGroupColor[1], defaultGroupColor[2])
{ };

//=============================================================================================================

EVENTSLIB::RgbColor::RgbColor(const uchar rRhs, const uchar gRhs, const uchar bRhs)
: RgbColor(rRhs, gRhs, bRhs, defaultGroupTransparency)
{ };

//=============================================================================================================

EVENTSLIB::RgbColor::RgbColor(const uchar rRhs, const uchar gRhs,
                              const uchar bRhs, const uchar aRhs)
: r(rRhs)
, g(gRhs)
, b(bRhs)
, a(aRhs)
{ };

//=============================================================================================================

EVENTSLIB::EventGroup::EventGroup(const EVENTSLIB::EventGroup& g)
: id(g.id)
, name(g.name)
, color(g.color)
, order(g.order)
{

}

//=============================================================================================================

EVENTSLIB::EventGroup::EventGroup(const EVENTSINTERNAL::EventGroupINT& g)
: id(g.getId())
, name(g.getName())
, color(g.getColor())
, order(g.getOrder())
{

}

//=============================================================================================================

EventGroupINT::EventGroupINT(const char* name)
: EventGroupINT(std::string(name))
{

}

//=============================================================================================================

EventGroupINT::EventGroupINT(std::string&& name)
: m_sName(std::move(name))
, m_Id(0)
, m_order(0)
{
    std::srand(std::time(NULL));

    setRandomColor();
}

//=============================================================================================================

EventGroupINT::EventGroupINT(idNum id, const std::string& name)
: m_sName(name)
, m_Id(id)
, m_order(0)
{
    std::srand(std::time(NULL));

    setRandomColor();
}

//=============================================================================================================

EventGroupINT::EventGroupINT(idNum id, const std::string& name,
                       const EVENTSLIB::RgbColor& color)
: m_sName(name)
, m_Id(id)
, m_order(0)
{
    std::srand(std::time(NULL));

    setColor(color);
}

//=============================================================================================================

void EventGroupINT::setColor(const EVENTSLIB::RgbColor& color)
{
    m_Color = color;
}

//=============================================================================================================

EVENTSLIB::RgbColor EventGroupINT::getColor() const
{
    return m_Color;
}

//=============================================================================================================

void EventGroupINT::setRandomColor()
{
    m_Color.r = rand() % 256;
    m_Color.g = rand() % 256;
    m_Color.b = rand() % 256;
}

//=============================================================================================================

const std::string& EventGroupINT::getName() const
{
    return m_sName;
}

//=============================================================================================================

void EventGroupINT::setName(const std::string &sName)
{
    m_sName = sName;
}

//=============================================================================================================

idNum EventGroupINT::getId() const
{
    return m_Id;
}

//=============================================================================================================

std::string EventGroupINT::getDescription() const
{
    return m_sDescription;
}

//=============================================================================================================

int EventGroupINT::getOrder() const
{
    return m_order;
}

//=============================================================================================================

void EventGroupINT::setOrder(int order)
{
    m_order = order;
}

//=============================================================================================================

bool EventGroupINT::operator<(const EventGroupINT &groupRHS) const
{
    return m_Id < groupRHS.getId();
}
