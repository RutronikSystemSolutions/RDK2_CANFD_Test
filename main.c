/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RutDevKit-PSoC62_Arduino_ADC_DMA_PDL
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-10-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

void handle_error(void);
void CANFD_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_rx_buffer_t* pstcCanFDmsg);
void CanfdInterruptHandler(void);


const cy_stc_sysint_t irq_cfg =
{
    canfd_0_interrupts0_0_IRQn,
    3UL
};
cy_stc_canfd_context_t canfd_context;
uint32_t bmp_data[16] =
		{
				0x00000000,
				0xDEADBEAF,
				0xCAFEBABE,
				0xFACEFEED,
				0x11111111,
				0x22222222,
				0x33333333,
				0x44444444,
				0x55555555,
				0x66666666,
				0x77777777,
				0x88888888,
				0x99999999,
				0xAAAAAAAA,
				0xBBBBBBBB,
				0xCCCCCCCC,
		};

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Initialize CANFD Driver Stand-By pin */
    result = cyhal_gpio_init( CANFD_STB, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK2 CAN FD Test Example has Started.\r\n");

    if(CY_CANFD_SUCCESS != Cy_CANFD_Init (CANFD0, 0, &CAN_FD_config, &canfd_context))
    {
        handle_error();
    }
    /* Enables the configuration changes to set Test mode */
    Cy_CANFD_ConfigChangesEnable(CANFD0, 0);
    /* Sets the Test mode configuration */
    Cy_CANFD_TestModeConfig(CANFD0, 0, CY_CANFD_TEST_MODE_EXTERNAL_LOOP_BACK);
    /* Disables the configuration changes */
    Cy_CANFD_ConfigChangesDisable(CANFD0, 0);
    /* Hook the interrupt service routine and enable the interrupt */
    (void) Cy_SysInt_Init(&irq_cfg, &CanfdInterruptHandler);
    NVIC_EnableIRQ(canfd_0_interrupts0_0_IRQn);

    /* Prepare data to send */
    memcpy(CAN_FD_dataBuffer_0, bmp_data, 64);

    for (;;)
    {
        /* Sends the prepared data using tx buffer 1 and waits for 1000ms */
        Cy_CANFD_UpdateAndTransmitMsgBuffer(CANFD0, 0u, &CAN_FD_txBuffer_0, 0u, &canfd_context);
    }
}

/* CANFD reception callback */
void CANFD_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_rx_buffer_t* pstcCanFDmsg)
{
    (void)bRxFifoMsg;
    (void)u8MsgBufOrRxFifoNum;

    /* Only for data frames */
    if(0 == pstcCanFDmsg->r0_f->rtr)
    {
    	cyhal_gpio_toggle(LED1);
    	memset(bmp_data, 0x00, sizeof(bmp_data));
    	memcpy(bmp_data, pstcCanFDmsg->data_area_f, 64);

    	/*Print the data received from the loopback*/
    	printf("\r\n Data received in loop back mode: \r\n");
        for(uint32_t index = 0; index < 16; index++)
        {
            printf("0x%08X ", (unsigned int)bmp_data[index]);

            if(0u == ((index + 1) % 4))
            {
                printf("\r\n");
            }
        }

        /*Increase the data header*/
        CAN_FD_dataBuffer_0[CAN_FD_DATA_0] += 1;
    }
}

/* CANFD interrupt handler */
void CanfdInterruptHandler(void)
{
    /* Just call the IRQ handler with the current channel number and context */
    Cy_CANFD_IrqHandler(CANFD0, 0, &canfd_context);
    CyDelay(1000);
}

void handle_error(void)
{
    /* Disable all interrupts. */
   __disable_irq();
   cyhal_gpio_write(LED1, CYBSP_LED_STATE_OFF);
   cyhal_gpio_write(LED2, CYBSP_LED_STATE_ON);
   CY_ASSERT(0);
}

/* [] END OF FILE */
