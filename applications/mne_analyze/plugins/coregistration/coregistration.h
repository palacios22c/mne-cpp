//=============================================================================================================
/**
 * @file     coregistration.h
 * @author   Ruben Dörfel <doerfelruben@aol.com>
 * @since    0.1.6
 * @date     August, 2020
 *
 * @section  LICENSE
 *
 * Copyright (C) 2020, Gabriel Motta. All rights reserved.
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
 * @brief    Contains the declaration of the CoRegistration class.
 *
 */

#ifndef COREGISTRATION_H
#define COREGISTRATION_H

//=============================================================================================================
// INCLUDES
//=============================================================================================================

#include "coregistration_global.h"
#include <anShared/Interfaces/IPlugin.h>

#include <fiff/fiff_coord_trans.h>
#include <fiff/fiff_dig_point.h>
#include <fiff/fiff_dig_point_set.h>

//=============================================================================================================
// QT INCLUDES
//=============================================================================================================

#include <QtWidgets>
#include <QtCore/QtPlugin>
#include <QDebug>
#include <QPointer>

//=============================================================================================================
// FORWARD DECLARATIONS
//=============================================================================================================

namespace ANSHAREDLIB {
    class Communicator;
    class AbstractModel;
}

namespace DISPLIB {
    class CoregSettingsView;
}

namespace MNELIB {
    class MNEBem;
}

namespace FIFFLIB {
    class FiffDigPoint;
    class FiffDigPointSet;
    class FiffCoordTrans;
}
//=============================================================================================================
// DEFINE NAMESPACE SURFERPLUGIN
//=============================================================================================================

namespace COREGISTRATIONPLUGIN
{

//=============================================================================================================
/**
 * CoRegistration Plugin
 *
 * @brief The CoRegistration class provides a view with all currently loaded models.
 */
class COREGISTRATIONSHARED_EXPORT CoRegistration : public ANSHAREDLIB::IPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ansharedlib/1.0" FILE "coregistration.json") //New Qt5 Plugin system replaces Q_EXPORT_PLUGIN2 macro
    // Use the Q_INTERFACES() macro to tell Qt's meta-object system about the interfaces
    Q_INTERFACES(ANSHAREDLIB::IPlugin)

public:
    //=========================================================================================================
    /**
     * Constructs a CoRegistration.
     */
    CoRegistration();

    //=========================================================================================================
    /**
     * Destroys the CoRegistration.
     */
    virtual ~CoRegistration() override;

    // IPlugin functions
    virtual QSharedPointer<IPlugin> clone() const override;
    virtual void init() override;
    virtual void unload() override;
    virtual QString getName() const override;
    virtual QMenu* getMenu() override;
    virtual QDockWidget* getControl() override;
    virtual QWidget* getView() override;
    virtual void handleEvent(QSharedPointer<ANSHAREDLIB::Event> e) override;
    virtual QVector<ANSHAREDLIB::EVENT_TYPE> getEventSubscriptions() const override;

private:

    //=========================================================================================================
    /**
     * Updates the dropdown display for selecting the bem model
     */
    void updateBemList(QSharedPointer<ANSHAREDLIB::AbstractModel> pNewModel);

    //=========================================================================================================
    /**
     * Connected to GUI dropdown to select bem based on bem name input.
     *
     * @param[in] text  name of bem selected in the GUI
     */
    void onChangeSelectedBem(const QString &sText);

    //=========================================================================================================
    /**
     * Call this funciton whenever new digitzers were loaded.
     *
     * @param[in] lDigitzers    The new digitzers.
     * @param[in] sFilePath     The file path to the new digitzers.
     */
    void onDigitizersChanged(const QString& sFilePath);

    //=========================================================================================================
    /**
     * Call this funciton whenever new fiducials were loaded.
     *
     */
    void onFiducialsChanged(const QString& sFilePath);

    //=========================================================================================================
    /**
     * Perform the actual Coregistration with the ICP algorithm
     *
     */
    void onFitICP();

    //=========================================================================================================
    /**
     * Fit fiducials as initial alignment for further coregistration with ICP algorithm
     *
     */
    void onFitFiducials();

    QVector<QSharedPointer<ANSHAREDLIB::AbstractModel>>     m_vecBemModels;         /** Vector with all available Bem Models */

    int                                                     m_iCurrentBem;          /**< Bem index to use. 9999 for current selection */

    QSharedPointer<MNELIB::MNEBem>                          m_pBem;                 /**< The currently selected Bem model */
    FIFFLIB::FiffDigPointSet                                m_digFidHead;           /**< The currently selected digitizer set */
    FIFFLIB::FiffDigPointSet                                m_digFidMri;            /**< The currently selected mri fiducials */
    FIFFLIB::FiffCoordTrans                                 m_transHeadMri;         /**< The resulting head-mri transformation */
    QPointer<ANSHAREDLIB::Communicator>                     m_pCommu;

    DISPLIB::CoregSettingsView*                             m_pCoregSettingsView;   /**< Pointer to coreg GUI */

};

//=============================================================================================================
// INLINE DEFINITIONS
//=============================================================================================================

} // NAMESPACE

#endif // COREGISTRATION_H
