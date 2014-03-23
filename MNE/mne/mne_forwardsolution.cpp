//=============================================================================================================
/**
* @file     mne_forwardsolution.cpp
* @author   Christoph Dinh <chdinh@nmr.mgh.harvard.edu>;
*           Matti Hamalainen <msh@nmr.mgh.harvard.edu>
* @version  1.0
* @date     July, 2012
*
* @section  LICENSE
*
* Copyright (C) 2012, Christoph Dinh and Matti Hamalainen. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that
* the following conditions are met:
*     * Redistributions of source code must retain the above copyright notice, this list of conditions and the
*       following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
*       the following disclaimer in the documentation and/or other materials provided with the distribution.
*     * Neither the name of the Massachusetts General Hospital nor the names of its contributors may be used
*       to endorse or promote products derived from this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MASSACHUSETTS GENERAL HOSPITAL BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*
* @brief     MNEForwardSolution class implementation
*
*/

//*************************************************************************************************************
//=============================================================================================================
// INCLUDES
//=============================================================================================================

#include "mne_forwardsolution.h"


//*************************************************************************************************************
//=============================================================================================================
// FIFF INCLUDES
//=============================================================================================================

#include <fs/colortable.h>
#include <fs/label.h>
#include <utils/mnemath.h>
#include <utils/kmeans.h>


//*************************************************************************************************************
//=============================================================================================================
// STL INCLUDES
//=============================================================================================================

#include <iostream>
#include <QtConcurrent>
#include <QFuture>


//*************************************************************************************************************
//=============================================================================================================
// USED NAMESPACES
//=============================================================================================================

using namespace MNELIB;
using namespace UTILSLIB;
using namespace FSLIB;


//*************************************************************************************************************
//=============================================================================================================
// DEFINE MEMBER METHODS
//=============================================================================================================

MNEForwardSolution::MNEForwardSolution()
: source_ori(-1)
, surf_ori(false)
, coord_frame(-1)
, nsource(-1)
, nchan(-1)
, sol(new FiffNamedMatrix)
, sol_grad(new FiffNamedMatrix)
//, mri_head_t(NULL)
//, src(NULL)
, source_rr(MatrixX3f::Zero(0,3))
, source_nn(MatrixX3f::Zero(0,3))
{

}


//*************************************************************************************************************

MNEForwardSolution::MNEForwardSolution(QIODevice &p_IODevice, bool force_fixed, bool surf_ori, const QStringList& include, const QStringList& exclude, bool bExcludeBads)
: source_ori(-1)
, surf_ori(surf_ori)
, coord_frame(-1)
, nsource(-1)
, nchan(-1)
, sol(new FiffNamedMatrix)
, sol_grad(new FiffNamedMatrix)
//, mri_head_t(NULL)
//, src(NULL)
, source_rr(MatrixX3f::Zero(0,3))
, source_nn(MatrixX3f::Zero(0,3))
{
    if(!read(p_IODevice, *this, force_fixed, surf_ori, include, exclude, bExcludeBads))
    {
        printf("\tForward solution not found.\n");//ToDo Throw here
        return;
    }
}


//*************************************************************************************************************

MNEForwardSolution::MNEForwardSolution(const MNEForwardSolution &p_MNEForwardSolution)
: info(p_MNEForwardSolution.info)
, source_ori(p_MNEForwardSolution.source_ori)
, surf_ori(p_MNEForwardSolution.surf_ori)
, coord_frame(p_MNEForwardSolution.coord_frame)
, nsource(p_MNEForwardSolution.nsource)
, nchan(p_MNEForwardSolution.nchan)
, sol(p_MNEForwardSolution.sol)
, sol_grad(p_MNEForwardSolution.sol_grad)
, mri_head_t(p_MNEForwardSolution.mri_head_t)
, src(p_MNEForwardSolution.src)
, source_rr(p_MNEForwardSolution.source_rr)
, source_nn(p_MNEForwardSolution.source_nn)
{

}


//*************************************************************************************************************

MNEForwardSolution::~MNEForwardSolution()
{

}


//*************************************************************************************************************

void MNEForwardSolution::clear()
{
    info.clear();
    source_ori = -1;
    surf_ori = false;
    coord_frame = -1;
    nsource = -1;
    nchan = -1;
    sol = FiffNamedMatrix::SDPtr(new FiffNamedMatrix());
    sol_grad = FiffNamedMatrix::SDPtr(new FiffNamedMatrix());
    mri_head_t.clear();
    src.clear();
    source_rr = MatrixX3f(0,3);
    source_nn = MatrixX3f(0,3);
}


//*************************************************************************************************************

MNEForwardSolution MNEForwardSolution::cluster_forward_solution(AnnotationSet &p_AnnotationSet, qint32 p_iClusterSize)
{
    MNEForwardSolution p_fwdOut = MNEForwardSolution(*this);

    //
    // Check consisty
    //
//    for(qint32 h = 0; h < this->src.hemispheres.size(); ++h )//obj.sizeForwardSolution)
//    {
//        if(this->src[h]->vertno.rows() !=  t_listAnnotation[h]->getLabel()->rows())
//        {
//            printf("Error: Annotation doesn't fit to Forward Solution: Vertice number is different!");
//            return false;
//        }
//    }


//    //DEBUG
//    MatrixXd test(20,3);
//    test << 0.537667139546100, 1.83388501459509, -2.25884686100365,
//            0.862173320368121, 0.318765239858981, -1.30768829630527,
//            -0.433592022305684, 0.342624466538650, 3.57839693972576,
//            2.76943702988488, -1.34988694015652, 3.03492346633185,
//            0.725404224946106, -0.0630548731896562, 0.714742903826096,
//            -0.204966058299775, -0.124144348216312, 1.48969760778547,
//            1.40903448980048, 1.41719241342961, 0.671497133608081,
//            -1.20748692268504, 0.717238651328839, 1.63023528916473,
//            0.488893770311789, 1.03469300991786, 0.726885133383238,
//            -0.303440924786016, 0.293871467096658, -0.787282803758638,
//            0.888395631757642, -1.14707010696915, -1.06887045816803,
//            -0.809498694424876, -2.94428416199490, 1.43838029281510,
//            0.325190539456198, -0.754928319169703, 1.37029854009523,
//            -1.71151641885370, -0.102242446085491, -0.241447041607358,
//            0.319206739165502, 0.312858596637428, -0.864879917324457,
//            -0.0300512961962686, -0.164879019209038, 0.627707287528727,
//            1.09326566903948, 1.10927329761440, -0.863652821988714,
//            0.0773590911304249, -1.21411704361541, -1.11350074148676,
//            -0.00684932810334806, 1.53263030828475, -0.769665913753682,
//            0.371378812760058, -0.225584402271252, 1.11735613881447;

//    std::cout << test << std::endl;

//    VectorXi idx;
//    MatrixXd ctrs;
//    VectorXd sumd;
//    MatrixXd D;

//    KMeans testK(QString("cityblock"), QString("sample"), 5);//QString("sqeuclidean")//QString("sample")

//    testK.calculate(test, 2, idx, ctrs, sumd, D);

//    std::cout << "idx" << std::endl << idx << std::endl;
//    std::cout << "ctrs" << std::endl << ctrs << std::endl;
//    std::cout << "sumd" << std::endl << sumd << std::endl;
//    std::cout << "D" << std::endl << D << std::endl;
//    //DEBUG END


    KMeans t_kMeans(QString("cityblock"), QString("sample"), 5);//QString("sqeuclidean")//QString("sample")//cityblock
    MatrixXd t_LF_new;

    qint32 count;
    qint32 offset;

    for(qint32 h = 0; h < this->src.size(); ++h )//obj.sizeForwardSolution)
    {
        count = 0;
        offset = 0;

        // Offset for continuous indexing;
        if(h > 0)
            for(qint32 j = 0; j < h; ++j)
                offset += this->src[j].nuse;

        if(h == 0)
            printf("Cluster Left Hemisphere\n");
        else
            printf("Cluster Right Hemisphere\n");

        Colortable t_CurrentColorTable = p_AnnotationSet[h].getColortable();
        VectorXi label_ids = t_CurrentColorTable.getLabelIds();

        // Get label ids for every vertex
        VectorXi vertno_labeled = VectorXi::Zero(this->src[h].vertno.rows());

        //ToDo make this more universal -> using Label instead of annotations - obsolete when using Labels
        for(qint32 i = 0; i < vertno_labeled.rows(); ++i)
            vertno_labeled[i] = p_AnnotationSet[h].getLabelIds()[this->src[h].vertno[i]];

        //iterate over labels
        MatrixXd t_LF_partial;
        //ToDo OpenMP
        for (qint32 i = 0; i < label_ids.rows(); ++i)
        {

            if (label_ids[i] != 0)
            {
                QString curr_name = t_CurrentColorTable.struct_names[i];//obj.label2AtlasName(label(i));
                printf("\tCluster %d / %li %s...", i+1, label_ids.rows(), curr_name.toUtf8().constData());

                //
                // Get source space indeces
                //
                VectorXi idcs = VectorXi::Zero(vertno_labeled.rows());
                qint32 c = 0;

                //Select ROIs //change this use label info with a hash tabel
                for(qint32 j = 0; j < vertno_labeled.rows(); ++j)
                {
                    if(vertno_labeled[j] == label_ids[i])
                    {
                        idcs[c] = j;
                        ++c;
                    }
                }
                idcs.conservativeResize(c);

                //get selected LF
                MatrixXd t_LF(this->sol->data.rows(), idcs.rows()*3);

                for(qint32 j = 0; j < idcs.rows(); ++j)
                    t_LF.block(0, j*3, t_LF.rows(), 3) = this->sol->data.block(0, (idcs[j]+offset)*3, t_LF.rows(), 3);

                qint32 nSens = t_LF.rows();
                qint32 nSources = t_LF.cols()/3;
                qint32 nClusters = 0;

                if (nSources > 0)
                {
                    nClusters = ceil((double)nSources/(double)p_iClusterSize);

                    printf("%d Cluster(s)... ", nClusters);

                    t_LF_partial = MatrixXd::Zero(nSens,nClusters*3);

                    // Reshape Input data -> sources rows; sensors columns
                    MatrixXd t_sensLF(t_LF.cols()/3, 3*nSens);
                    for(qint32 j = 0; j < nSens; ++j)
                    {
                        for(qint32 k = 0; k < t_sensLF.rows(); ++k)
                            t_sensLF.block(k,j*3,1,3) = t_LF.block(j,k*3,1,3);
                    }

                    // Kmeans Reduction
                    VectorXi roiIdx;
                    MatrixXd ctrs;
                    VectorXd sumd;
                    MatrixXd D;

                    t_kMeans.calculate(t_sensLF, nClusters, roiIdx, ctrs, sumd, D);

                    //
                    // Assign the centroid for each cluster to the partial LF
                    //
                    for(qint32 j = 0; j < nSens; ++j)
                        for(qint32 k = 0; k < nClusters; ++k)
                            t_LF_partial.block(j, k*3, 1, 3) = ctrs.block(k,j*3,1,3);

                    //
                    // Get cluster indizes and its distances to the centroid
                    //
                    for(qint32 j = 0; j < nClusters; ++j)
                    {
                        VectorXi clusterIdcs = VectorXi::Zero(roiIdx.rows());
                        VectorXd clusterDistance = VectorXd::Zero(roiIdx.rows());
                        qint32 nClusterIdcs = 0;
                        for(qint32 k = 0; k < roiIdx.rows(); ++k)
                        {
                            if(roiIdx[k] == j)
                            {
                                clusterIdcs[nClusterIdcs] = idcs[k];
                                clusterDistance[nClusterIdcs] = D(k,j);
                                ++nClusterIdcs;
                            }
                        }
                        clusterIdcs.conservativeResize(nClusterIdcs);
                        p_fwdOut.src[h].cluster_info.clusterVertnos.append(clusterIdcs);
                        p_fwdOut.src[h].cluster_info.clusterDistances.append(clusterDistance);
                        p_fwdOut.src[h].cluster_info.clusterLabelIds.append(label_ids[i]);
                    }

                    //
                    // Assign partial LF to new LeadField
                    //
                    if(t_LF_partial.rows() > 0 && t_LF_partial.cols() > 0)
                    {
                        t_LF_new.conservativeResize(t_LF_partial.rows(), t_LF_new.cols() + t_LF_partial.cols());
                        t_LF_new.block(0, t_LF_new.cols() - t_LF_partial.cols(), t_LF_new.rows(), t_LF_partial.cols()) = t_LF_partial;

                        // Map the centroids to the closest rr
                        for(qint32 k = 0; k < nClusters; ++k)
                        {
                            qint32 j = 0;

                            double sqec = sqrt((t_LF.block(0, j*3, t_LF.rows(), 3) - t_LF_partial.block(0, k*3, t_LF_partial.rows(), 3)).array().pow(2).sum());
                            double sqec_min = sqec;
                            qint32 j_min = j;
                            for(qint32 j = 1; j < idcs.rows(); ++j)
                            {
                                sqec = sqrt((t_LF.block(0, j*3, t_LF.rows(), 3) - t_LF_partial.block(0, k*3, t_LF_partial.rows(), 3)).array().pow(2).sum());
                                if(sqec < sqec_min)
                                {
                                    sqec_min = sqec;
                                    j_min = j;
                                }
                            }

                            // Take the closest coordinates
                            qint32 sel_idx = idcs[j_min];
                            //ToDo store this in cluster info
//                            p_fwdOut.src[h].rr.row(count) = this->src[h].rr.row(sel_idx);
//                            p_fwdOut.src[h].nn.row(count) = MatrixXd::Zero(1,3);

                            p_fwdOut.src[h].vertno[count] = this->src[h].vertno[sel_idx];

                            ++count;
                        }
                    }
                    printf("[done]\n");
                }
                else
                {
                    printf("failed! Label contains no sources.\n");
                }
            }
        }

        //
        // Assemble new hemisphere information
        //
//ToDo store this in cluster info
//        p_fwdOut.src[h].rr.conservativeResize(count, 3);
//        p_fwdOut.src[h].nn.conservativeResize(count, 3);
        p_fwdOut.src[h].vertno.conservativeResize(count);

//        std::cout << "Vertno hemisphere:" << h << std::endl << p_fwdOut.src[h].vertno << std::endl;


//        p_fwdOut.src[h].nuse_tri = 0;
//        p_fwdOut.src[h].use_tris = MatrixX3i(0,3);


//        if(p_fwdOut.src[h].rr.rows() > 0 && p_fwdOut.src[h].rr.cols() > 0)
//        {
//            if(h == 0)
//            {
//                p_fwdOut.source_rr = MatrixX3d(0,3);
//                p_fwdOut.source_nn = MatrixX3d(0,3);
//            }

//            p_fwdOut.source_rr.conservativeResize(p_fwdOut.source_rr.rows() + p_fwdOut.src[h].rr.rows(),3);
//            p_fwdOut.source_rr.block(p_fwdOut.source_rr.rows() -  p_fwdOut.src[h].rr.rows(), 0,  p_fwdOut.src[h].rr.rows(), 3) = p_fwdOut.src[h].rr;

//            p_fwdOut.source_nn.conservativeResize(p_fwdOut.source_nn.rows() + p_fwdOut.src[h].nn.rows(),3);
//            p_fwdOut.source_nn.block(p_fwdOut.source_nn.rows() -  p_fwdOut.src[h].nn.rows(), 0,  p_fwdOut.src[h].nn.rows(), 3) = p_fwdOut.src[h].nn;
//        }

        printf("[done]\n");
    }

    //
    // Put it all together
    //
    p_fwdOut.sol->data = t_LF_new;
    p_fwdOut.sol->ncol = t_LF_new.cols();

    p_fwdOut.nsource = p_fwdOut.sol->ncol/3;

    return p_fwdOut;
}


//*************************************************************************************************************

MNEForwardSolution MNEForwardSolution::cluster_forward_solution_ccr(AnnotationSet &p_AnnotationSet, qint32 p_iClusterSize)
{
    MNEForwardSolution p_fwdOut = MNEForwardSolution(*this);

    //
    // Check consisty
    //
//    for(qint32 h = 0; h < this->src.hemispheres.size(); ++h )//obj.sizeForwardSolution)
//    {
//        if(this->src[h]->vertno.rows() !=  t_listAnnotation[h]->getLabel()->rows())
//        {
//            printf("Error: Annotation doesn't fit to Forward Solution: Vertice number is different!");
//            return false;
//        }
//    }





    KMeans t_kMeans(QString("cityblock"), QString("sample"), 5);//QString("sqeuclidean")//QString("sample")//cityblock
    MatrixXd t_LF_new;

    qint32 count;
    qint32 offset;

    for(qint32 h = 0; h < this->src.size(); ++h )
    {

        count = 0;
        offset = 0;

        // Offset for continuous indexing;
        if(h > 0)
            for(qint32 j = 0; j < h; ++j)
                offset += this->src[j].nuse;

        if(h == 0)
            printf("Cluster Left Hemisphere\n");
        else
            printf("Cluster Right Hemisphere\n");

        Colortable t_CurrentColorTable = p_AnnotationSet[h].getColortable();
        VectorXi label_ids = t_CurrentColorTable.getLabelIds();

        // Get label ids for every vertex
        VectorXi vertno_labeled = VectorXi::Zero(this->src[h].vertno.rows());

        //ToDo make this more universal -> using Label instead of annotations - obsolete when using Labels
        for(qint32 i = 0; i < vertno_labeled.rows(); ++i)
            vertno_labeled[i] = p_AnnotationSet[h].getLabelIds()[this->src[h].vertno[i]];

        //Qt Concurrent List
        QList<RegionDataIn> m_qListRegionDataIn;

        for (qint32 i = 0; i < label_ids.rows(); ++i)
        {
            if (label_ids[i] != 0)
            {
                QString curr_name = t_CurrentColorTable.struct_names[i];//obj.label2AtlasName(label(i));
                printf("\tCluster %d / %li %s...", i+1, label_ids.rows(), curr_name.toUtf8().constData());

                //
                // Get source space indeces
                //
                VectorXi idcs = VectorXi::Zero(vertno_labeled.rows());
                qint32 c = 0;

                //Select ROIs //change this use label info with a hash tabel
                for(qint32 j = 0; j < vertno_labeled.rows(); ++j)
                {
                    if(vertno_labeled[j] == label_ids[i])
                    {
                        idcs[c] = j;
                        ++c;
                    }
                }
                idcs.conservativeResize(c);

                //get selected LF
                MatrixXd t_LF(this->sol->data.rows(), idcs.rows()*3);

                for(qint32 j = 0; j < idcs.rows(); ++j)
                    t_LF.block(0, j*3, t_LF.rows(), 3) = this->sol->data.block(0, (idcs[j]+offset)*3, t_LF.rows(), 3);

                qint32 nSens = t_LF.rows();
                qint32 nSources = t_LF.cols()/3;

                if (nSources > 0)
                {
                    RegionDataIn t_sensG;

                    t_sensG.idcs = idcs;
                    t_sensG.iLabelIdxIn = i;
                    t_sensG.nClusters = ceil((double)nSources/(double)p_iClusterSize);

                    t_sensG.matRoiGOrig = t_LF;

                    printf("%d Cluster(s)... ", t_sensG.nClusters);

                    // Reshape Input data -> sources rows; sensors columns
                    t_sensG.matRoiG = MatrixXd(t_LF.cols()/3, 3*nSens);

                    for(qint32 j = 0; j < nSens; ++j)
                    {
                        for(qint32 k = 0; k < t_sensG.matRoiG.rows(); ++k)
                            t_sensG.matRoiG.block(k,j*3,1,3) = t_LF.block(j,k*3,1,3);
                    }

                    m_qListRegionDataIn.append(t_sensG);

                    printf("[added]\n");
                }
                else
                {
                    printf("failed! Label contains no sources.\n");
                }
            }
        }

        //
        // Calculate clusters
        //
        printf("Clustering... ");
        QFuture< RegionDataOut > res;
        res = QtConcurrent::mapped(m_qListRegionDataIn, &RegionDataIn::cluster);
        res.waitForFinished();
        printf("[done]\n");


        //
        // Assign results
        //
        MatrixXd t_LF_partial;

        qint32 nClusters;
        qint32 nSens;
        QFuture<RegionDataOut>::const_iterator itOut;
        QList<RegionDataIn>::const_iterator itIn;
        itIn = m_qListRegionDataIn.begin();
        for (itOut = res.constBegin(); itOut != res.constEnd(); ++itOut)
        {
            nClusters = itOut->ctrs.rows();
            nSens = itOut->ctrs.cols()/3;
            t_LF_partial = MatrixXd::Zero(nSens, nClusters*3);

//            std::cout << "Number of Clusters: " << nClusters << " x " << nSens << std::endl;//itOut->iLabelIdcsOut << std::endl;

            //
            // Assign the centroid for each cluster to the partial LF
            //
            for(qint32 j = 0; j < nSens; ++j)
                for(qint32 k = 0; k < nClusters; ++k)
                    t_LF_partial.block(j, k*3, 1, 3) = itOut->ctrs.block(k,j*3,1,3);

            //
            // Get cluster indizes and its distances to the centroid
            //
            for(qint32 j = 0; j < nClusters; ++j)
            {
                VectorXi clusterIdcs = VectorXi::Zero(itOut->roiIdx.rows());
                VectorXd clusterDistance = VectorXd::Zero(itOut->roiIdx.rows());
                qint32 nClusterIdcs = 0;
                for(qint32 k = 0; k < itOut->roiIdx.rows(); ++k)
                {
                    if(itOut->roiIdx[k] == j)
                    {
                        clusterIdcs[nClusterIdcs] = itIn->idcs[k];
                        clusterDistance[nClusterIdcs] = itOut->D(k,j);
                        ++nClusterIdcs;
                    }
                }
                clusterIdcs.conservativeResize(nClusterIdcs);
                p_fwdOut.src[h].cluster_info.clusterVertnos.append(clusterIdcs);
                p_fwdOut.src[h].cluster_info.clusterDistances.append(clusterDistance);
                p_fwdOut.src[h].cluster_info.clusterLabelIds.append(label_ids[itOut->iLabelIdxOut]);
            }


            //
            // Assign partial LF to new LeadField
            //
            if(t_LF_partial.rows() > 0 && t_LF_partial.cols() > 0)
            {
                t_LF_new.conservativeResize(t_LF_partial.rows(), t_LF_new.cols() + t_LF_partial.cols());
                t_LF_new.block(0, t_LF_new.cols() - t_LF_partial.cols(), t_LF_new.rows(), t_LF_partial.cols()) = t_LF_partial;

                // Map the centroids to the closest rr
                for(qint32 k = 0; k < nClusters; ++k)
                {
                    qint32 j = 0;

                    double sqec = sqrt((itIn->matRoiGOrig.block(0, j*3, itIn->matRoiGOrig.rows(), 3) - t_LF_partial.block(0, k*3, t_LF_partial.rows(), 3)).array().pow(2).sum());
                    double sqec_min = sqec;
                    qint32 j_min = 0;
                    for(qint32 j = 1; j < itIn->idcs.rows(); ++j)
                    {
                        sqec = sqrt((itIn->matRoiGOrig.block(0, j*3, itIn->matRoiGOrig.rows(), 3) - t_LF_partial.block(0, k*3, t_LF_partial.rows(), 3)).array().pow(2).sum());

                        if(sqec < sqec_min)
                        {
                            sqec_min = sqec;
                            j_min = j;
                        }
                    }

                    // Take the closest coordinates
                    qint32 sel_idx = itIn->idcs[j_min];
                    //ToDo store this in cluster info
//                    p_fwdOut.src[h].rr.row(count) = this->src[h].rr.row(sel_idx);
//                    p_fwdOut.src[h].nn.row(count) = MatrixXd::Zero(1,3);

                    p_fwdOut.src[h].vertno[count] = this->src[h].vertno[sel_idx];

                    ++count;
                }
            }

            ++itIn;
        }

        //
        // Assemble new hemisphere information
        //
//ToDo store this in cluster info
//        p_fwdOut.src[h].rr.conservativeResize(count, 3);
//        p_fwdOut.src[h].nn.conservativeResize(count, 3);
        p_fwdOut.src[h].vertno.conservativeResize(count);

//        std::cout << "Vertno hemisphere:" << h << std::endl << p_fwdOut.src[h].vertno << std::endl;


//        p_fwdOut.src[h].nuse_tri = 0;
//        p_fwdOut.src[h].use_tris = MatrixX3i(0,3);


//        if(p_fwdOut.src[h].rr.rows() > 0 && p_fwdOut.src[h].rr.cols() > 0)
//        {
//            if(h == 0)
//            {
//                p_fwdOut.source_rr = MatrixX3d(0,3);
//                p_fwdOut.source_nn = MatrixX3d(0,3);
//            }

//            p_fwdOut.source_rr.conservativeResize(p_fwdOut.source_rr.rows() + p_fwdOut.src[h].rr.rows(),3);
//            p_fwdOut.source_rr.block(p_fwdOut.source_rr.rows() -  p_fwdOut.src[h].rr.rows(), 0,  p_fwdOut.src[h].rr.rows(), 3) = p_fwdOut.src[h].rr;

//            p_fwdOut.source_nn.conservativeResize(p_fwdOut.source_nn.rows() + p_fwdOut.src[h].nn.rows(),3);
//            p_fwdOut.source_nn.block(p_fwdOut.source_nn.rows() -  p_fwdOut.src[h].nn.rows(), 0,  p_fwdOut.src[h].nn.rows(), 3) = p_fwdOut.src[h].nn;
//        }

//        printf("[done]\n");
    }

    //
    // Put it all together
    //
    p_fwdOut.sol->data = t_LF_new;
    p_fwdOut.sol->ncol = t_LF_new.cols();

    p_fwdOut.nsource = p_fwdOut.sol->ncol/3;

    return p_fwdOut;
}


//*************************************************************************************************************

FiffCov MNEForwardSolution::compute_depth_prior(const MatrixXd &Gain, const FiffInfo &gain_info, bool is_fixed_ori, double exp, double limit, const MatrixXd &patch_areas, bool limit_depth_chs)
{
    printf("\tCreating the depth weighting matrix...\n");

    MatrixXd G(Gain);
    // If possible, pick best depth-weighting channels
    if(limit_depth_chs)
        MNEForwardSolution::restrict_gain_matrix(G, gain_info);

    VectorXd d;
    // Compute the gain matrix
    if(is_fixed_ori)
    {
        d = (G.array().square()).rowwise().sum(); //ToDo: is this correct - is G squared?
//            d = np.sum(G ** 2, axis=0)
    }
    else
    {
        qint32 n_pos = G.cols() / 3;
        d = VectorXd::Zero(n_pos);
        MatrixXd Gk;
        for (qint32 k = 0; k < n_pos; ++k)
        {
            Gk = G.block(0,3*k, G.rows(), 3);
            JacobiSVD<MatrixXd> svd(Gk.transpose()*Gk);
            d[k] = svd.singularValues().maxCoeff();
        }
    }

    // ToDo Currently the fwd solns never have "patch_areas" defined
    if(patch_areas.cols() > 0)
    {
//            d /= patch_areas ** 2
        printf("\tToDo!!!!! >>> Patch areas taken into account in the depth weighting\n");
    }

    qint32 n_limit;
    VectorXd w = d.cwiseInverse();
    VectorXd ws = w;
    VectorXd wpp;
    MNEMath::sort<double>(ws, false);
    double weight_limit = pow(limit, 2);
    if (!limit_depth_chs)
    {
        // match old mne-python behavor
        qint32 ind;
        ws.minCoeff(&ind);
        n_limit = ind;
        limit = ws[ind] * weight_limit;
    }
    else
    {
        // match C code behavior
        limit = ws[ws.size()-1];
        qint32 ind;
        n_limit = d.size();
        if (ws[ws.size()-1] > weight_limit * ws[0])
        {
            double th = weight_limit * ws[0];
            for(qint32 i = 0; i < ws.size(); ++i)
            {
                if(ws[i] > th)
                {
                    ind = i;
                    break;
                }
            }
            limit = ws[ind];
            n_limit = ind;
        }
    }

    printf("\tlimit = %d/%li = %f", n_limit + 1, d.size(), sqrt(limit / ws[0]));
    double scale = 1.0 / limit;
    printf("\tscale = %g exp = %g", scale, exp);

    VectorXd t_w = w.array() / limit;
    for(qint32 i = 0; i < t_w.size(); ++i)
        t_w[i] = t_w[i] > 1 ? 1 : t_w[i];
    wpp = t_w.array().pow(exp);

    FiffCov depth_prior;
    if(is_fixed_ori)
        depth_prior.data = wpp;
    else
    {
        depth_prior.data.resize(wpp.rows()*3, 1);
        qint32 idx = 0;
        double v;
        for(qint32 i = 0; i < wpp.rows(); ++i)
        {
            idx = i*3;
            v = wpp[i];
            depth_prior.data(idx, 0) = v;
            depth_prior.data(idx+1, 0) = v;
            depth_prior.data(idx+2, 0) = v;
        }
    }

    depth_prior.kind = FIFFV_MNE_DEPTH_PRIOR_COV;
    depth_prior.diag = true;
    depth_prior.dim = depth_prior.data.rows();
    depth_prior.nfree = 1;

    return depth_prior;
}


//*************************************************************************************************************

FiffCov MNEForwardSolution::compute_orient_prior(float loose)
{
    bool is_fixed_ori = this->isFixedOrient();
    qint32 n_sources = this->sol->data.cols();

    if (0 <= loose && loose <= 1)
    {
        if(loose < 1 && !this->surf_ori)
        {
            printf("\tForward operator is not oriented in surface coordinates. loose parameter should be None not %f.", loose);//ToDo Throw here
            loose = 1;
            printf("\tSetting loose to %f.\n", loose);
        }

        if(is_fixed_ori)
        {
            printf("\tIgnoring loose parameter with forward operator with fixed orientation.\n");
            loose = 0.0;
        }
    }
    else
    {
        if(loose < 0 || loose > 1)
        {
            qWarning("Warning: Loose value should be in interval [0,1] not %f.\n", loose);
            loose = loose > 1 ? 1 : 0;
            printf("Setting loose to %f.\n", loose);
        }
    }

    FiffCov orient_prior;
    orient_prior.data = VectorXd::Ones(n_sources);
    if(!is_fixed_ori && (0 <= loose && loose <= 1))
    {
        printf("\tApplying loose dipole orientations. Loose value of %f.\n", loose);
        for(qint32 i = 0; i < n_sources; i+=3)
            orient_prior.data.block(i,0,2,1).array() *= loose;

        orient_prior.kind = FIFFV_MNE_ORIENT_PRIOR_COV;
        orient_prior.diag = true;
        orient_prior.dim = orient_prior.data.size();
        orient_prior.nfree = 1;
    }
    return orient_prior;
}


//*************************************************************************************************************

MNEForwardSolution MNEForwardSolution::pick_channels(const QStringList& include, const QStringList& exclude) const
{
    MNEForwardSolution fwd(*this);

    if(include.size() == 0 && exclude.size() == 0)
        return fwd;

    RowVectorXi sel = FiffInfo::pick_channels(fwd.sol->row_names, include, exclude);

    // Do we have something?
    quint32 nuse = sel.size();

    if (nuse == 0)
    {
        printf("Nothing remains after picking. Returning original forward solution.\n");
        return fwd;
    }
    printf("\t%d out of %d channels remain after picking\n", nuse, fwd.nchan);

    //   Pick the correct rows of the forward operator
    MatrixXd newData(nuse, fwd.sol->data.cols());
    for(quint32 i = 0; i < nuse; ++i)
        newData.row(i) = fwd.sol->data.row(sel[i]);

    fwd.sol->data = newData;
    fwd.sol->nrow = nuse;

    QStringList ch_names;
    for(qint32 i = 0; i < sel.cols(); ++i)
        ch_names << fwd.sol->row_names[sel(i)];
    fwd.nchan = nuse;
    fwd.sol->row_names = ch_names;

    QList<FiffChInfo> chs;
    for(qint32 i = 0; i < sel.cols(); ++i)
        chs.append(fwd.info.chs[sel(i)]);
    fwd.info.chs = chs;
    fwd.info.nchan = nuse;

    QStringList bads;
    for(qint32 i = 0; i < fwd.info.bads.size(); ++i)
        if(ch_names.contains(fwd.info.bads[i]))
            bads.append(fwd.info.bads[i]);
    fwd.info.bads = bads;

    if(!fwd.sol_grad->isEmpty())
    {
        newData.resize(nuse, fwd.sol_grad->data.cols());
        for(quint32 i = 0; i < nuse; ++i)
            newData.row(i) = fwd.sol_grad->data.row(sel[i]);
        fwd.sol_grad->data = newData;
        fwd.sol_grad->nrow = nuse;
        QStringList row_names;
        for(qint32 i = 0; i < sel.cols(); ++i)
            row_names << fwd.sol_grad->row_names[sel(i)];
        fwd.sol_grad->row_names = row_names;
    }

    return fwd;
}


//*************************************************************************************************************

MNEForwardSolution MNEForwardSolution::pick_regions(const QList<Label> &p_qListLabels) const
{
    VectorXi selVertices;

    qint32 iSize = 0;
    for(qint32 i = 0; i < p_qListLabels.size(); ++i)
    {
        VectorXi currentSelection;
        this->src.label_src_vertno_sel(p_qListLabels[i], currentSelection);

        selVertices.conservativeResize(iSize+currentSelection.size());
        selVertices.block(iSize,0,currentSelection.size(),1) = currentSelection;
        iSize = selVertices.size();
    }

    MNEMath::sort(selVertices, false);

    MNEForwardSolution selectedFwd(*this);

    MatrixX3f rr(selVertices.size(),3);
    MatrixX3f nn(selVertices.size(),3);

    for(qint32 i = 0; i < selVertices.size(); ++i)
    {
        rr.block(i, 0, 1, 3) = selectedFwd.source_rr.row(selVertices[i]);
        nn.block(i, 0, 1, 3) = selectedFwd.source_nn.row(selVertices[i]);
    }

    selectedFwd.source_rr = rr;
    selectedFwd.source_nn = nn;

    VectorXi selSolIdcs = tripletSelection(selVertices);
    MatrixXd G(selectedFwd.sol->data.rows(),selSolIdcs.size());
//    selectedFwd.sol_grad; //ToDo
    qint32 rows = G.rows();

    for(qint32 i = 0; i < selSolIdcs.size(); ++i)
        G.block(0, i, rows, 1) = selectedFwd.sol->data.col(selSolIdcs[i]);

    selectedFwd.sol->data = G;
    selectedFwd.sol->nrow = selectedFwd.sol->data.rows();
    selectedFwd.sol->ncol = selectedFwd.sol->data.cols();
    selectedFwd.nsource = selectedFwd.sol->ncol / 3;

    selectedFwd.src = selectedFwd.src.pick_regions(p_qListLabels);

    return selectedFwd;
}


//*************************************************************************************************************

MNEForwardSolution MNEForwardSolution::pick_types(bool meg, bool eeg, const QStringList& include, const QStringList& exclude) const
{
    RowVectorXi sel = info.pick_types(meg, eeg, false, include, exclude);

    QStringList include_ch_names;
    for(qint32 i = 0; i < sel.cols(); ++i)
        include_ch_names << info.ch_names[sel[i]];

    return this->pick_channels(include_ch_names);
}


//*************************************************************************************************************

void MNEForwardSolution::prepare_forward(const FiffInfo &p_info, const FiffCov &p_noise_cov, bool p_pca, FiffInfo &p_outFwdInfo, MatrixXd &gain, FiffCov &p_outNoiseCov, MatrixXd &p_outWhitener, qint32 &p_outNumNonZero) const
{
    QStringList fwd_ch_names, ch_names;
    for(qint32 i = 0; i < this->info.chs.size(); ++i)
        fwd_ch_names << this->info.chs[i].ch_name;

    ch_names.clear();
    for(qint32 i = 0; i < p_info.chs.size(); ++i)
        if(     !p_info.bads.contains(p_info.chs[i].ch_name)
            &&  !p_noise_cov.bads.contains(p_info.chs[i].ch_name)
            &&  fwd_ch_names.contains(p_info.chs[i].ch_name))
            ch_names << p_info.chs[i].ch_name;

    qint32 n_chan = ch_names.size();
    printf("Computing inverse operator with %d channels.\n", n_chan);

    //
    //   Handle noise cov
    //
    p_outNoiseCov = p_noise_cov.prepare_noise_cov(p_info, ch_names);

    //   Omit the zeroes due to projection
    p_outNumNonZero = 0;
    VectorXi t_vecNonZero = VectorXi::Zero(n_chan);
    for(qint32 i = 0; i < p_outNoiseCov.eig.rows(); ++i)
    {
        if(p_outNoiseCov.eig[i] > 0)
        {
            t_vecNonZero[p_outNumNonZero] = i;
            ++p_outNumNonZero;
        }
    }
    if(p_outNumNonZero > 0)
        t_vecNonZero.conservativeResize(p_outNumNonZero);

    if(p_outNumNonZero > 0)
    {
        if (p_pca)
        {
            qWarning("Warning in MNEForwardSolution::prepare_forward: if (p_pca) havent been debugged.");
            p_outWhitener = MatrixXd::Zero(n_chan, p_outNumNonZero);
            // Rows of eigvec are the eigenvectors
            for(qint32 i = 0; i < p_outNumNonZero; ++i)
                p_outWhitener.col(t_vecNonZero[i]) = p_outNoiseCov.eigvec.col(t_vecNonZero[i]).array() / sqrt(p_outNoiseCov.eig(t_vecNonZero[i]));
            printf("\tReducing data rank to %d.\n", p_outNumNonZero);
        }
        else
        {
            printf("Creating non pca whitener.\n");
            p_outWhitener = MatrixXd::Zero(n_chan, n_chan);
            for(qint32 i = 0; i < p_outNumNonZero; ++i)
                p_outWhitener(t_vecNonZero[i],t_vecNonZero[i]) = 1.0 / sqrt(p_outNoiseCov.eig(t_vecNonZero[i]));
            // Cols of eigvec are the eigenvectors
            p_outWhitener *= p_outNoiseCov.eigvec;
        }
    }

    VectorXi fwd_idx = VectorXi::Zero(ch_names.size());
    VectorXi info_idx = VectorXi::Zero(ch_names.size());
    qint32 idx;
    qint32 count_fwd_idx = 0;
    qint32 count_info_idx = 0;
    for(qint32 i = 0; i < ch_names.size(); ++i)
    {
        idx = fwd_ch_names.indexOf(ch_names[i]);
        if(idx > -1)
        {
            fwd_idx[count_fwd_idx] = idx;
            ++count_fwd_idx;
        }
        idx = p_info.ch_names.indexOf(ch_names[i]);
        if(idx > -1)
        {
            info_idx[count_info_idx] = idx;
            ++count_info_idx;
        }
    }
    fwd_idx.conservativeResize(count_fwd_idx);
    info_idx.conservativeResize(count_info_idx);

    gain.resize(count_fwd_idx, this->sol->data.cols());
    for(qint32 i = 0; i < count_fwd_idx; ++i)
        gain.row(i) = this->sol->data.row(fwd_idx[i]);

    p_outFwdInfo = p_info.pick_info(info_idx);

    printf("\tTotal rank is %d\n", p_outNumNonZero);
}


//*************************************************************************************************************

bool MNEForwardSolution::read(QIODevice& p_IODevice, MNEForwardSolution& fwd, bool force_fixed, bool surf_ori, const QStringList& include, const QStringList& exclude, bool bExcludeBads)
{
    FiffStream::SPtr t_pStream(new FiffStream(&p_IODevice));
    FiffDirTree t_Tree;
    QList<FiffDirEntry> t_Dir;

    printf("Reading forward solution from %s...\n", t_pStream->streamName().toUtf8().constData());
    if(!t_pStream->open(t_Tree, t_Dir))
        return false;
    //
    //   Find all forward solutions
    //
    QList<FiffDirTree> fwds = t_Tree.dir_tree_find(FIFFB_MNE_FORWARD_SOLUTION);

    if (fwds.size() == 0)
    {
        t_pStream->device()->close();
        std::cout << "No forward solutions in " << t_pStream->streamName().toUtf8().constData(); // ToDo throw error
        return false;
    }
    //
    //   Parent MRI data
    //
    QList<FiffDirTree> parent_mri = t_Tree.dir_tree_find(FIFFB_MNE_PARENT_MRI_FILE);
    if (parent_mri.size() == 0)
    {
        t_pStream->device()->close();
        std::cout << "No parent MRI information in " << t_pStream->streamName().toUtf8().constData(); // ToDo throw error
        return false;
    }

    MNESourceSpace t_SourceSpace;// = NULL;
    if(!MNESourceSpace::readFromStream(t_pStream, true, t_Tree, t_SourceSpace))
    {
        t_pStream->device()->close();
        std::cout << "Could not read the source spaces\n"; // ToDo throw error
        //ToDo error(me,'Could not read the source spaces (%s)',mne_omit_first_line(lasterr));
        return false;
    }

    for(qint32 k = 0; k < t_SourceSpace.size(); ++k)
        t_SourceSpace[k].id = MNESourceSpace::find_source_space_hemi(t_SourceSpace[k]);

    //
    //   Bad channel list
    //
    QStringList bads;
    if(bExcludeBads)
    {
        bads = t_pStream->read_bad_channels(t_Tree);
        if(bads.size() > 0)
        {
            printf("\t%d bad channels ( ",bads.size());
            for(qint32 i = 0; i < bads.size(); ++i)
                printf("\"%s\" ", bads[i].toLatin1().constData());
            printf(") read\n");
        }
    }

    //
    //   Locate and read the forward solutions
    //
    FiffTag::SPtr t_pTag;
    FiffDirTree megnode;
    FiffDirTree eegnode;
    for(qint32 k = 0; k < fwds.size(); ++k)
    {
        if(!fwds[k].find_tag(t_pStream.data(), FIFF_MNE_INCLUDED_METHODS, t_pTag))
        {
            t_pStream->device()->close();
            std::cout << "Methods not listed for one of the forward solutions\n"; // ToDo throw error
            return false;
        }
        if (*t_pTag->toInt() == FIFFV_MNE_MEG)
        {
            printf("MEG solution found\n");
            megnode = fwds[k];
        }
        else if(*t_pTag->toInt() == FIFFV_MNE_EEG)
        {
            printf("EEG solution found\n");
            eegnode = fwds.at(k);
        }
    }

    MNEForwardSolution megfwd;
    QString ori;
    if (read_one(t_pStream.data(), megnode, megfwd))
    {
        if (megfwd.source_ori == FIFFV_MNE_FIXED_ORI)
            ori = QString("fixed");
        else
            ori = QString("free");
        printf("\tRead MEG forward solution (%d sources, %d channels, %s orientations)\n", megfwd.nsource,megfwd.nchan,ori.toUtf8().constData());
    }
    MNEForwardSolution eegfwd;
    if (read_one(t_pStream.data(), eegnode, eegfwd))
    {
        if (eegfwd.source_ori == FIFFV_MNE_FIXED_ORI)
            ori = QString("fixed");
        else
            ori = QString("free");
        printf("\tRead EEG forward solution (%d sources, %d channels, %s orientations)\n", eegfwd.nsource,eegfwd.nchan,ori.toUtf8().constData());
    }

    //
    //   Merge the MEG and EEG solutions together
    //
    fwd.clear();

    if (!megfwd.isEmpty() && !eegfwd.isEmpty())
    {
        if (megfwd.sol->data.cols() != eegfwd.sol->data.cols() ||
                megfwd.source_ori != eegfwd.source_ori ||
                megfwd.nsource != eegfwd.nsource ||
                megfwd.coord_frame != eegfwd.coord_frame)
        {
            t_pStream->device()->close();
            std::cout << "The MEG and EEG forward solutions do not match\n"; // ToDo throw error
            return false;
        }

        fwd = MNEForwardSolution(megfwd);
        fwd.sol->data = MatrixXd(megfwd.sol->nrow + eegfwd.sol->nrow, megfwd.sol->ncol);

        fwd.sol->data.block(0,0,megfwd.sol->nrow,megfwd.sol->ncol) = megfwd.sol->data;
        fwd.sol->data.block(megfwd.sol->nrow,0,eegfwd.sol->nrow,eegfwd.sol->ncol) = eegfwd.sol->data;
        fwd.sol->nrow = megfwd.sol->nrow + eegfwd.sol->nrow;
        fwd.sol->row_names.append(eegfwd.sol->row_names);

        if (!fwd.sol_grad->isEmpty())
        {
            fwd.sol_grad->data.resize(megfwd.sol_grad->data.rows() + eegfwd.sol_grad->data.rows(), megfwd.sol_grad->data.cols());

            fwd.sol->data.block(0,0,megfwd.sol_grad->data.rows(),megfwd.sol_grad->data.cols()) = megfwd.sol_grad->data;
            fwd.sol->data.block(megfwd.sol_grad->data.rows(),0,eegfwd.sol_grad->data.rows(),eegfwd.sol_grad->data.cols()) = eegfwd.sol_grad->data;

            fwd.sol_grad->nrow      = megfwd.sol_grad->nrow + eegfwd.sol_grad->nrow;
            fwd.sol_grad->row_names.append(eegfwd.sol_grad->row_names);
        }
        fwd.nchan  = megfwd.nchan + eegfwd.nchan;
        printf("\tMEG and EEG forward solutions combined\n");
    }
    else if (!megfwd.isEmpty())
        fwd = megfwd; //new MNEForwardSolution(megfwd);//not copied for the sake of speed
    else
        fwd = eegfwd; //new MNEForwardSolution(eegfwd);//not copied for the sake of speed

    //
    //   Get the MRI <-> head coordinate transformation
    //
    if(!parent_mri[0].find_tag(t_pStream.data(), FIFF_COORD_TRANS, t_pTag))
    {
        t_pStream->device()->close();
        std::cout << "MRI/head coordinate transformation not found\n"; // ToDo throw error
        return false;
    }
    else
    {
        fwd.mri_head_t = t_pTag->toCoordTrans();

        if (fwd.mri_head_t.from != FIFFV_COORD_MRI || fwd.mri_head_t.to != FIFFV_COORD_HEAD)
        {
            fwd.mri_head_t.invert_transform();
            if (fwd.mri_head_t.from != FIFFV_COORD_MRI || fwd.mri_head_t.to != FIFFV_COORD_HEAD)
            {
                t_pStream->device()->close();
                std::cout << "MRI/head coordinate transformation not found\n"; // ToDo throw error
                return false;
            }
        }
    }

    //
    // get parent MEG info -> from python package
    //
    t_pStream->read_meas_info_base(t_Tree, fwd.info);


    t_pStream->device()->close();

    //
    //   Transform the source spaces to the correct coordinate frame
    //   if necessary
    //
    if (fwd.coord_frame != FIFFV_COORD_MRI && fwd.coord_frame != FIFFV_COORD_HEAD)
    {
        std::cout << "Only forward solutions computed in MRI or head coordinates are acceptable";
        return false;
    }

    //
    qint32 nuse = 0;
    t_SourceSpace.transform_source_space_to(fwd.coord_frame,fwd.mri_head_t);
    for(qint32 k = 0; k < t_SourceSpace.size(); ++k)
        nuse += t_SourceSpace[k].nuse;

    if (nuse != fwd.nsource)
        throw("Source spaces do not match the forward solution.\n");

    printf("\tSource spaces transformed to the forward solution coordinate frame\n");
    fwd.src = t_SourceSpace; //not new MNESourceSpace(t_SourceSpace); for sake of speed
    //
    //   Handle the source locations and orientations
    //
    if (fwd.isFixedOrient() || force_fixed == true)
    {
        nuse = 0;
        fwd.source_rr = MatrixXf::Zero(fwd.nsource,3);
        fwd.source_nn = MatrixXf::Zero(fwd.nsource,3);
        for(qint32 k = 0; k < t_SourceSpace.size();++k)
        {
            for(qint32 q = 0; q < t_SourceSpace[k].nuse; ++q)
            {
                fwd.source_rr.block(q,0,1,3) = t_SourceSpace[k].rr.block(t_SourceSpace[k].vertno(q),0,1,3);
                fwd.source_nn.block(q,0,1,3) = t_SourceSpace[k].nn.block(t_SourceSpace[k].vertno(q),0,1,3);
            }
            nuse += t_SourceSpace[k].nuse;
        }
        //
        //   Modify the forward solution for fixed source orientations
        //
        if (fwd.source_ori != FIFFV_MNE_FIXED_ORI)
        {
            printf("\tChanging to fixed-orientation forward solution...");

            MatrixXd tmp = fwd.source_nn.transpose().cast<double>();
            SparseMatrix<double>* fix_rot = MNEMath::make_block_diag(tmp,1);
            fwd.sol->data *= (*fix_rot);
            fwd.sol->ncol  = fwd.nsource;
            fwd.source_ori = FIFFV_MNE_FIXED_ORI;

            if (!fwd.sol_grad->isEmpty())
            {
                SparseMatrix<double> t_matKron;
                SparseMatrix<double> t_eye(3,3);
                for (qint32 i = 0; i < 3; ++i)
                    t_eye.insert(i,i) = 1.0f;
                t_matKron = kroneckerProduct(*fix_rot,t_eye);//kron(fix_rot,eye(3));
                fwd.sol_grad->data *= t_matKron;
                fwd.sol_grad->ncol   = 3*fwd.nsource;
            }
            delete fix_rot;
            printf("[done]\n");
        }
    }
    else if (surf_ori)
    {
        //
        //   Rotate the local source coordinate systems
        //
        printf("\tConverting to surface-based source orientations...");

        bool use_ave_nn = false;
        if(t_SourceSpace[0].patch_inds.size() > 0)
        {
            use_ave_nn = true;
            printf("\tAverage patch normals will be employed in the rotation to the local surface coordinates...\n");
        }

        nuse = 0;
        qint32 pp = 0;
        fwd.source_rr = MatrixXf::Zero(fwd.nsource,3);
        fwd.source_nn = MatrixXf::Zero(fwd.nsource*3,3);

        qWarning("Warning source_ori: Rotating the source coordinate system haven't been verified --> Singular Vectors U are different from MATLAB!");

        for(qint32 k = 0; k < t_SourceSpace.size();++k)
        {

            for (qint32 q = 0; q < t_SourceSpace[k].nuse; ++q)
                fwd.source_rr.block(q+nuse,0,1,3) = t_SourceSpace[k].rr.block(t_SourceSpace[k].vertno(q),0,1,3);

            for (qint32 p = 0; p < t_SourceSpace[k].nuse; ++p)
            {
                //
                //  Project out the surface normal and compute SVD
                //
                Vector3f nn;
                if(use_ave_nn)
                {
                    VectorXi t_vIdx = t_SourceSpace[k].pinfo[t_SourceSpace[k].patch_inds[p]];
                    Matrix3Xf t_nn(3, t_vIdx.size());
                    for(qint32 i = 0; i < t_vIdx.size(); ++i)
                        t_nn.col(i) = t_SourceSpace[k].nn.block(t_vIdx[i],0,1,3).transpose();
                    nn = t_nn.rowwise().sum();
                    nn.array() /= nn.norm();
                }
                else
                    nn = t_SourceSpace[k].nn.block(t_SourceSpace[k].vertno(p),0,1,3).transpose();

                Matrix3f tmp = Matrix3f::Identity(nn.rows(), nn.rows()) - nn*nn.transpose();

                JacobiSVD<MatrixXf> t_svd(tmp, Eigen::ComputeThinU);
                //Sort singular values and singular vectors
                VectorXf t_s = t_svd.singularValues();
                MatrixXf U = t_svd.matrixU();
                MNEMath::sort<float>(t_s, U);

                //
                //  Make sure that ez is in the direction of nn
                //
                if ((nn.transpose() * U.block(0,2,3,1))(0,0) < 0)
                    U *= -1;
                fwd.source_nn.block(pp, 0, 3, 3) = U.transpose();
                pp += 3;
            }
            nuse += t_SourceSpace[k].nuse;
        }
        MatrixXd tmp = fwd.source_nn.transpose().cast<double>();
        SparseMatrix<double>* surf_rot = MNEMath::make_block_diag(tmp,3);

        fwd.sol->data *= *surf_rot;

        if (!fwd.sol_grad->isEmpty())
        {
            SparseMatrix<double> t_matKron;
            SparseMatrix<double> t_eye(3,3);
            for (qint32 i = 0; i < 3; ++i)
                t_eye.insert(i,i) = 1.0f;
            t_matKron = kroneckerProduct(*surf_rot,t_eye);//kron(surf_rot,eye(3));
            fwd.sol_grad->data *= t_matKron;
        }
        delete surf_rot;
        printf("[done]\n");
    }
    else
    {
        printf("\tCartesian source orientations...");
        nuse = 0;
        fwd.source_rr = MatrixXf::Zero(fwd.nsource,3);
        for(qint32 k = 0; k < t_SourceSpace.size(); ++k)
        {
            for (qint32 q = 0; q < t_SourceSpace[k].nuse; ++q)
                fwd.source_rr.block(q+nuse,0,1,3) = t_SourceSpace[k].rr.block(t_SourceSpace[k].vertno(q),0,1,3);

            nuse += t_SourceSpace[k].nuse;
        }

        MatrixXf t_ones = MatrixXf::Ones(fwd.nsource,1);
        Matrix3f t_eye = Matrix3f::Identity();
        fwd.source_nn = kroneckerProduct(t_ones,t_eye);

        printf("[done]\n");
    }

    //
    //   Do the channel selection
    //
    QStringList exclude_bads = exclude;
    if (bads.size() > 0)
    {
        for(qint32 k = 0; k < bads.size(); ++k)
            if(!exclude_bads.contains(bads[k],Qt::CaseInsensitive))
                exclude_bads << bads[k];
    }

    fwd.surf_ori = surf_ori;
    fwd = fwd.pick_channels(include, exclude_bads);

//    //
//    //   Do the channel selection - OLD VERSION
//    //
//    if (include.size() > 0 || exclude.size() > 0 || bads.size() > 0)
//    {
//        //
//        //   First do the channels to be included
//        //
//        RowVectorXi pick;
//        if (include.size() == 0)
//            pick = RowVectorXi::Ones(fwd.nchan);
//        else
//        {
//            pick = RowVectorXi::Zero(fwd.nchan);
//            for(qint32 k = 0; k < include.size(); ++k)
//            {
//                QList<int> c;
//                for(qint32 l = 0; l < fwd.sol->row_names.size(); ++l)
//                    if(fwd.sol->row_names.at(l).contains(include.at(k),Qt::CaseInsensitive))
//                        pick(l) = 1;

////                    c = strmatch(include{k},fwd.sol->row_names,'exact');
////                    for p = 1:length(c)
////                        pick(c(p)) = 1;
////                    end
//            }
//        }
//        //
//        //   Then exclude what needs to be excluded
//        //
//        if (exclude.size() > 0)
//        {
//            for(qint32 k = 0; k < exclude.size(); ++k)
//            {
//                QList<int> c;
//                for(qint32 l = 0; l < fwd.sol->row_names.size(); ++l)
//                    if(fwd.sol->row_names.at(l).contains(exclude.at(k),Qt::CaseInsensitive))
//                        pick(l) = 0;

////                    c = strmatch(exclude{k},fwd.sol->row_names,'exact');
////                    for p = 1:length(c)
////                        pick(c(p)) = 0;
////                    end
//            }
//        }
//        if (bads.size() > 0)
//        {
//            for(qint32 k = 0; k < bads.size(); ++k)
//            {
//                QList<int> c;
//                for(qint32 l = 0; l < fwd.sol->row_names.size(); ++l)
//                    if(fwd.sol->row_names.at(l).contains(bads.at(k),Qt::CaseInsensitive))
//                        pick(l) = 0;

////                    c = strmatch(bads{k},fwd.sol->row_names,'exact');
////                    for p = 1:length(c)
////                        pick(c(p)) = 0;
////                    end
//            }
//        }
//        //
//        //   Do we have something?
//        //
//        nuse = pick.sum();
//        if (nuse == 0)
//            throw("Nothing remains after picking");
//        //
//        //   Create the selector
//        //
//        qint32 p = 0;
//        MatrixXd t_solData(nuse,fwd.sol->data.cols());
//        QStringList t_solRowNames;

//        MatrixXd t_solGradData;// = NULL;
//        QStringList t_solGradRowNames;

//        QList<FiffChInfo> chs;


//        if (!fwd.sol_grad->isEmpty())
//            t_solGradData.resize(nuse, fwd.sol_grad->data.cols());

//        for(qint32 k = 0; k < fwd.nchan; ++k)
//        {
//            if(pick(k) > 0)
//            {
//                t_solData.block(p, 0, 1, fwd.sol->data.cols()) = fwd.sol->data.block(k, 0, 1, fwd.sol->data.cols());
//                t_solRowNames.append(fwd.sol->row_names[k]);

//                if (!fwd.sol_grad->isEmpty())
//                {
//                    t_solGradData.block(p, 0, 1, fwd.sol_grad->data.cols()) = fwd.sol_grad->data.block(k, 0, 1, fwd.sol_grad->data.cols());
//                    t_solGradRowNames.append(fwd.sol_grad->row_names[k]);
//                }

//                chs.append(fwd.info.chs[k]);

//                ++p;
//            }
//        }
//        printf("\t%d out of %d channels remain after picking\n",nuse,fwd.nchan);
//        //
//        //   Pick the correct rows of the forward operator
//        //
//        fwd.nchan          = nuse;
////        if(fwd.sol->data)
////            delete fwd.sol->data;
//        fwd.sol->data      = t_solData;
//        fwd.sol->nrow      = nuse;
//        fwd.sol->row_names = t_solRowNames;

//        if (!fwd.sol_grad->isEmpty())
//        {
//            fwd.sol_grad->data      = t_solGradData;
//            fwd.sol_grad->nrow      = nuse;
//            fwd.sol_grad->row_names = t_solGradRowNames;
//        }

//        fwd.info.chs = chs;
//    }

    //garbage collecting
    t_pStream->device()->close();

    return true;
}


//*************************************************************************************************************

bool MNEForwardSolution::read_one(FiffStream* p_pStream, const FiffDirTree& p_Node, MNEForwardSolution& one)
{
    //
    //   Read all interesting stuff for one forward solution
    //
    if(p_Node.isEmpty())
        return false;

    one.clear();
    FiffTag::SPtr t_pTag;

    if(!p_Node.find_tag(p_pStream, FIFF_MNE_SOURCE_ORIENTATION, t_pTag))
    {
        p_pStream->device()->close();
        std::cout << "Source orientation tag not found."; //ToDo: throw error.
        return false;
    }

    one.source_ori = *t_pTag->toInt();

    if(!p_Node.find_tag(p_pStream, FIFF_MNE_COORD_FRAME, t_pTag))
    {
        p_pStream->device()->close();
        std::cout << "Coordinate frame tag not found."; //ToDo: throw error.
        return false;
    }

    one.coord_frame = *t_pTag->toInt();

    if(!p_Node.find_tag(p_pStream, FIFF_MNE_SOURCE_SPACE_NPOINTS, t_pTag))
    {
        p_pStream->device()->close();
        std::cout << "Number of sources not found."; //ToDo: throw error.
        return false;
    }

    one.nsource = *t_pTag->toInt();

    if(!p_Node.find_tag(p_pStream, FIFF_NCHAN, t_pTag))
    {
        p_pStream->device()->close();
        printf("Number of channels not found."); //ToDo: throw error.
        return false;
    }

    one.nchan = *t_pTag->toInt();

    if(p_pStream->read_named_matrix(p_Node, FIFF_MNE_FORWARD_SOLUTION, *one.sol.data()))
        one.sol->transpose_named_matrix();
    else
    {
        p_pStream->device()->close();
        printf("Forward solution data not found ."); //ToDo: throw error.
        //error(me,'Forward solution data not found (%s)',mne_omit_first_line(lasterr));
        return false;
    }

    if(p_pStream->read_named_matrix(p_Node, FIFF_MNE_FORWARD_SOLUTION_GRAD, *one.sol_grad.data()))
        one.sol_grad->transpose_named_matrix();
    else
        one.sol_grad->clear();


    if (one.sol->data.rows() != one.nchan ||
            (one.sol->data.cols() != one.nsource && one.sol->data.cols() != 3*one.nsource))
    {
        p_pStream->device()->close();
        printf("Forward solution matrix has wrong dimensions.\n"); //ToDo: throw error.
        //error(me,'Forward solution matrix has wrong dimensions');
        return false;
    }
    if (!one.sol_grad->isEmpty())
    {
        if (one.sol_grad->data.rows() != one.nchan ||
                (one.sol_grad->data.cols() != 3*one.nsource && one.sol_grad->data.cols() != 3*3*one.nsource))
        {
            p_pStream->device()->close();
            printf("Forward solution gradient matrix has wrong dimensions.\n"); //ToDo: throw error.
            //error(me,'Forward solution gradient matrix has wrong dimensions');
        }
    }
    return true;
}


//*************************************************************************************************************

void MNEForwardSolution::restrict_gain_matrix(MatrixXd &G, const FiffInfo &info)
{
    // Figure out which ones have been used
    if(info.chs.size() != G.rows())
    {
        printf("Error G.rows() and length of info.chs do not match: %li != %i", G.rows(), info.chs.size()); //ToDo throw
        return;
    }

    RowVectorXi sel = info.pick_types(QString("grad"));
    if(sel.size() > 0)
    {
        for(qint32 i = 0; i < sel.size(); ++i)
            G.row(i) = G.row(sel[i]);
        G.conservativeResize(sel.size(), G.cols());
        printf("\t%li planar channels", sel.size());
    }
    else
    {
        sel = info.pick_types(QString("mag"));
        if (sel.size() > 0)
        {
            for(qint32 i = 0; i < sel.size(); ++i)
                G.row(i) = G.row(sel[i]);
            G.conservativeResize(sel.size(), G.cols());
            printf("\t%li magnetometer or axial gradiometer channels", sel.size());
        }
        else
        {
            sel = info.pick_types(false, true);
            if(sel.size() > 0)
            {
                for(qint32 i = 0; i < sel.size(); ++i)
                    G.row(i) = G.row(sel[i]);
                G.conservativeResize(sel.size(), G.cols());
                printf("\t%li EEG channels\n", sel.size());
            }
            else
                printf("Could not find MEG or EEG channels\n");
        }
    }
}


//*************************************************************************************************************

void MNEForwardSolution::to_fixed_ori()
{
    if(!this->surf_ori || this->isFixedOrient())
    {
        qWarning("Warning: Only surface-oriented, free-orientation forward solutions can be converted to fixed orientaton.\n");//ToDo: Throw here//qCritical//qFatal
        return;
    }
    qint32 count = 0;
    for(qint32 i = 2; i < this->sol->data.cols(); i += 3)
        this->sol->data.col(count) = this->sol->data.col(i);//ToDo: is this right? - just take z?
    this->sol->data.conservativeResize(this->sol->data.rows(), count);
    this->sol->ncol = this->sol->ncol / 3;
    this->source_ori = FIFFV_MNE_FIXED_ORI;
    printf("\tConverted the forward solution into the fixed-orientation mode.\n");
}
