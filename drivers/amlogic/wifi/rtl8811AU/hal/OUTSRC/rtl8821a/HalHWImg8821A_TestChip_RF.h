/******************************************************************************
*
* Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of version 2 of the GNU General Public License as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
*
*
******************************************************************************/

#if (RTL8821A_SUPPORT == 1)
#ifndef __INC_TC_RF_HW_IMG_8821A_H
#define __INC_TC_RF_HW_IMG_8821A_H

//static BOOLEAN CheckCondition(const u4Byte Condition, const u4Byte Hex);

/******************************************************************************
*                           RadioA.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_RadioA( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

/******************************************************************************
*                           TxPowerTrack_AP.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_TxPowerTrack_AP( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

/******************************************************************************
*                           TxPowerTrack_PCIE.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_TxPowerTrack_PCIE( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

/******************************************************************************
*                           TxPowerTrack_USB.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_TxPowerTrack_USB( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

/******************************************************************************
*                           TXPWR_LMT_8811AU_FEM.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_TXPWR_LMT_8811AU_FEM( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

/******************************************************************************
*                           TXPWR_LMT_8811AU_IPA.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_TXPWR_LMT_8811AU_IPA( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

/******************************************************************************
*                           TXPWR_LMT_8821A.TXT
******************************************************************************/

void
ODM_ReadAndConfig_TC_8821A_TXPWR_LMT_8821A( // TC: Test Chip, MP: MP Chip
	IN   PDM_ODM_T  pDM_Odm
);

#endif
#endif // end of HWIMG_SUPPORT
