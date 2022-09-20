
#include "SPI.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "clock_config.h"

void SPI_NOKIA_config(void)
{
	dspi_master_config_t masterConfig; //Variable/estructura para configurar el SPI
	uint32_t srcClock_Hz; //Variable para obtener la frecuencia

	CLOCK_SetSimSafeDivs(); //Establecemos lo del divisor de frecuencia
	CLOCK_EnableClock(kCLOCK_PortD); //Clock gating del puerto D
	CLOCK_EnableClock(kCLOCK_PortB); //Clock gating del puerto D
	
	//Configuramos los pines del puerto D como SPI
	PORT_SetPinMux(PORTB, 20u, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 12u, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTD, 13u, kPORT_MuxAlt2);
	//PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAlt2);

	//Master config
	masterConfig.whichCtar 								  = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate 					  = TRANSFER_BAUDRATE_NOKIA;
	masterConfig.ctarConfig.bitsPerFrame 				  = 8U;
	masterConfig.ctarConfig.cpol 						  = kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha 						  = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction 					  = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec 		  = 1U;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec 	  = 1U;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1U;

	masterConfig.whichPcs 			= kDSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

	masterConfig.enableContinuousSCK 		= false;
	masterConfig.enableRxFifoOverWrite 		= false;
	masterConfig.enableModifiedTimingFormat = false;
	masterConfig.samplePoint 				= kDSPI_SckToSin0Clock;

	srcClock_Hz = CLOCK_GetFreq(DSPI2_CLK_SRC);
	DSPI_MasterInit(SPI2, &masterConfig, srcClock_Hz);
}


