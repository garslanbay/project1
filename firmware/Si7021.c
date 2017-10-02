

#include "Si7021.h"

_Bool Si7013_Detect( tip *deviceId);

void Si7013_ReadNoHoldRHAndTemp(uint32_t *rhData ,int32_t *tData)
{
	uint8_t reg[2];
	
	 I2C_Read(0xE5,reg,2,rhData);

	*rhData = (((*rhData) * 15625L) >> 13) - 6000;
	
	I2C_Read(0xE3,reg,2,(uint32_t *)tData);
	 *tData = (((*tData) * 21965L) >> 13) - 46850; /* convert to milli-degC */
	
}

void I2C_Write(uint8_t Reg, int8_t* Val , uint8_t cnt)
{
	
	//Wait until I2C isn't busy
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(I2C1, Reg, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	//Ensure the transmit interrupted flag is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register we wish to write to
	I2C_SendData(I2C1, Reg);

	//Ensure that the transfer complete reload flag is
	//set, essentially a standard TC flag
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TCR) == RESET);

	//Now that the HMC5883L knows which register
	//we want to write to, send the address again
	//and ensure the I2C peripheral doesn't add
	//any start or stop conditions
	I2C_TransferHandling(I2C1, Reg,cnt, I2C_AutoEnd_Mode, I2C_No_StartStop);

	//Again, wait until the transmit interrupted flag is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the value you wish you write to the register
	while(cnt--)
	{
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
		I2C_SendData(I2C1, *Val);
		Val++;
		
	}

	//Wait for the stop flag to be set indicating
	//a stop condition has been sent
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);

	//Clear the stop flag for the next potential transfer
	I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
}

void I2C_Read(int8_t Reg, uint8_t *Data, uint8_t DCnt,uint32_t *hum)
{
	int8_t Cnt, SingleData = 0;
	int8_t temp;

	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	//Again, start another tranfer using the "transfer handling"
	//function, the end bit being set in software this time
	//round, generate a start condition and indicate you will
	//be writing data to the HMC device.
	I2C_TransferHandling(I2C1, SI7021_ADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	//Wait until the transmit interrupt status is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(I2C1, (uint8_t)Reg);

	//Wait until transfer is complete!
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);

	//As per, start another transfer, we want to read DCnt
	//amount of bytes. Generate a start condition and
	//indicate that we want to read.
	I2C_TransferHandling(I2C1, SI7021_ADDR, DCnt, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	//Read in DCnt pieces of data
	for(Cnt = 0; Cnt<DCnt; Cnt++)
	{
        //Wait until the RX register is full of luscious data!
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET); 
        //If we're only reading one byte, place that data direct into the 
        //SingleData variable. If we're reading more than 1 piece of data 
        //store in the array "Data" (a pointer from main) 		

					Data[Cnt] = I2C_ReceiveData(I2C1);
					temp=Data[0];
	}
		*hum=((uint32_t) Data[0] << 8) + (Data[1] & 0xfc);


     //Wait for the stop condition to be sent
     while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);

     //Clear the stop flag for next transfers
     I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);

     //Return a single piece of data if DCnt was
     //less than 1, otherwise 0 will be returned.


}
int32_t Si7013_GetFirmwareRevision(uint8_t addr, uint8_t *fwRev)
{
	
	int8_t Cnt, SingleData = 0;
	int8_t temp;

	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	//Again, start another tranfer using the "transfer handling"
	//function, the end bit being set in software this time
	//round, generate a start condition and indicate you will
	//be writing data to the HMC device.
	I2C_TransferHandling(I2C1, addr, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	//Wait until the transmit interrupt status is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(I2C1, (uint8_t)SI7013_READ_FWREV_1);
	
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(I2C1, (uint8_t) SI7013_READ_FWREV_2 );

	//Wait until transfer is complete!
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);

	//As per, start another transfer, we want to read DCnt
	//amount of bytes. Generate a start condition and
	//indicate that we want to read.
	I2C_TransferHandling(I2C1, addr, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);


        //Wait until the RX register is full of luscious data!
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET); 
        //If we're only reading one byte, place that data direct into the 
        //SingleData variable. If we're reading more than 1 piece of data 
        //store in the array "Data" (a pointer from main) 		

		fwRev[0] = I2C_ReceiveData(I2C1);


     //Wait for the stop condition to be sent
     while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);

     //Clear the stop flag for next transfers
     I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);

     //Return a single piece of data if DCnt was
     //less than 1, otherwise 0 will be returned.

		return 1;

}
_Bool Si7013_Detect( tip *deviceId)
{

	int8_t Cnt=0,Cnt2=0, SingleData = 0;
	int8_t temp;
	
	for(Cnt2=0;Cnt2<2;Cnt2++)
	{

	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);

	//Again, start another tranfer using the "transfer handling"
	//function, the end bit being set in software this time
	//round, generate a start condition and indicate you will
	//be writing data to the HMC device.
	I2C_TransferHandling(I2C1,SI7021_ADDR, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	//Wait until the transmit interrupt status is set
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(I2C1, (uint8_t)SI7013_READ_ID1_1);
	
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);

	//Send the address of the register you wish to read
	I2C_SendData(I2C1, (uint8_t) SI7013_READ_ID1_2 );

	//Wait until transfer is complete!
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);

	//As per, start another transfer, we want to read DCnt
	//amount of bytes. Generate a start condition and
	//indicate that we want to read.
	I2C_TransferHandling(I2C1, SI7021_ADDR, 8, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);


		if(Cnt2==0)
		{
			for(;Cnt<8;Cnt++)
			{
        //Wait until the RX register is full of luscious data!
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET); 
        //If we're only reading one byte, place that data direct into the 
        //SingleData variable. If we're reading more than 1 piece of data 
        //store in the array "Data" (a pointer from main) 		

				deviceId[Cnt].byte  = I2C_ReceiveData(I2C1);
			}
		}
		else
		{
			for(;Cnt<16;Cnt++)
			{
        //Wait until the RX register is full of luscious data!
        while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET); 
        //If we're only reading one byte, place that data direct into the 
        //SingleData variable. If we're reading more than 1 piece of data 
        //store in the array "Data" (a pointer from main) 		

				deviceId[Cnt].byte = I2C_ReceiveData(I2C1);
			}
		
		}


     //Wait for the stop condition to be sent
     while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);

     //Clear the stop flag for next transfers
     I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);

     //Return a single piece of data if DCnt was
     //less than 1, otherwise 0 will be returned.
	}

		return 1;



}

