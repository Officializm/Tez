import serial
import struct
import os
import sys
import glob

Flash_TAMAM                                         = 0x00
FLASH_HATA                                          = 0x01
Flash_MESGUL                                        = 0x02
Flash_HAL_TIMEOUT                                   = 0x03
Flash_GECERSIZ_ADDR                                 = 0x04


COMMAND_BL_GET_VER                                  =0x51
COMMAND_BL_FLASH_ERASE                              =0x56
COMMAND_BL_MEM_WRITE                                =0x57




COMMAND_BL_GET_VER_LEN                              =6
COMMAND_BL_FLASH_ERASE_LEN                          =8
COMMAND_BL_MEM_WRITE_LEN                            =11



verbose_mode = 1
mem_write_active =0



def decode_menu_command_code(command):
    ret_value = 0
    data_buf = []
    for i in range(255):
        data_buf.append(0)
    
    if(command  == 0 ):
        print("\n   Çıkış Yapılıyor...!")
        raise SystemExit
    elif(command == 1):
        print("\n   Command == > BOOTLOADER_GET_VERSION")
        COMMAND_BL_GET_VER_LEN              = 6
        data_buf[0] = COMMAND_BL_GET_VER_LEN-1 
        data_buf[1] = COMMAND_BL_GET_VER 
        crc32       = get_crc(data_buf,COMMAND_BL_GET_VER_LEN-4)
        crc32 = crc32 & 0xffffffff
        data_buf[2] = word_to_byte(crc32,1,1) 
        data_buf[3] = word_to_byte(crc32,2,1) 
        data_buf[4] = word_to_byte(crc32,3,1) 
        data_buf[5] = word_to_byte(crc32,4,1) 

        
        Write_to_serial_port(data_buf[0],1)
        for i in data_buf[1:COMMAND_BL_GET_VER_LEN]:
            Write_to_serial_port(i,COMMAND_BL_GET_VER_LEN-1)
        

        ret_value = read_bootloader_reply(data_buf[1])
        
 
    elif(command == 2):
        print("\n   Command == > BL_FLASH_ERASE")
        data_buf[0] = COMMAND_BL_FLASH_ERASE_LEN-1 
        data_buf[1] = COMMAND_BL_FLASH_ERASE 
        sector_num = input("\n   Sektorun numarasını giriniz(0-7 veya 0xFF) :")
        sector_num = int(sector_num, 16)
        if(sector_num != 0xff):
            nsec=int(input("\n  Silmek istediğiniz sektorlerin sayısını giriniz(max 8) :"))
        
        data_buf[2]= sector_num 
        data_buf[3]= nsec 

        crc32       = get_crc(data_buf,COMMAND_BL_FLASH_ERASE_LEN-4) 
        data_buf[4] = word_to_byte(crc32,1,1) 
        data_buf[5] = word_to_byte(crc32,2,1) 
        data_buf[6] = word_to_byte(crc32,3,1) 
        data_buf[7] = word_to_byte(crc32,4,1) 

        Write_to_serial_port(data_buf[0],1)
        
        for i in data_buf[1:COMMAND_BL_FLASH_ERASE_LEN]:
            Write_to_serial_port(i,COMMAND_BL_FLASH_ERASE_LEN-1)
        
        ret_value = read_bootloader_reply(data_buf[1])
        
    elif(command == 3):
        print("\n   Command == > BL_MEM_WRITE")
        bytes_remaining=0
        t_len_of_file=0
        bytes_so_far_sent = 0
        len_to_read=0
        base_mem_address=0

        data_buf[1] = COMMAND_BL_MEM_WRITE

        #First get the total number of bytes in the .bin file.
        t_len_of_file =calc_file_len()

        #keep opening the file
        open_the_file()

        bytes_remaining = t_len_of_file - bytes_so_far_sent

        base_mem_address = input("\n   Hafıza başlangıc  adresini giriniz :")
        base_mem_address = int(base_mem_address, 16)
        global mem_write_active
        while(bytes_remaining):
            mem_write_active=1
            if(bytes_remaining >= 128):
                len_to_read = 128
            else:
                len_to_read = bytes_remaining
            #get the bytes in to buffer by reading file
            for x in range(len_to_read):
                file_read_value = bin_file.read(1)
                file_read_value = bytearray(file_read_value)
                data_buf[7+x]= int(file_read_value[0])
            #read_the_file(&data_buf[7],len_to_read) 
            #print("\n   base mem address = \n",base_mem_address, hex(base_mem_address)) 

            #populate base mem address
            data_buf[2] = word_to_byte(base_mem_address,1,1)
            data_buf[3] = word_to_byte(base_mem_address,2,1)
            data_buf[4] = word_to_byte(base_mem_address,3,1)
            data_buf[5] = word_to_byte(base_mem_address,4,1)

            data_buf[6] = len_to_read

            #/* 1 byte len + 1 byte command code + 4 byte mem base address
            #* 1 byte payload len + len_to_read is amount of bytes read from file + 4 byte CRC
            #*/
            mem_write_cmd_total_len = COMMAND_BL_MEM_WRITE_LEN+len_to_read

            #first field is "len_to_follow"
            data_buf[0] =mem_write_cmd_total_len-1

            crc32       = get_crc(data_buf,mem_write_cmd_total_len-4)
            data_buf[7+len_to_read] = word_to_byte(crc32,1,1)
            data_buf[8+len_to_read] = word_to_byte(crc32,2,1)
            data_buf[9+len_to_read] = word_to_byte(crc32,3,1)
            data_buf[10+len_to_read] = word_to_byte(crc32,4,1)

            #update base mem address for the next loop
            base_mem_address+=len_to_read

            Write_to_serial_port(data_buf[0],1)
        
            for i in data_buf[1:mem_write_cmd_total_len]:
                Write_to_serial_port(i,mem_write_cmd_total_len-1)

            bytes_so_far_sent+=len_to_read
            bytes_remaining = t_len_of_file - bytes_so_far_sent
            print("\n   Gönderilen Byte:{0} -- Kalan Byte:{1}\n".format(bytes_so_far_sent,bytes_remaining)) 
        
            ret_value = read_bootloader_reply(data_buf[1])
        mem_write_active=0

  
    else:
        print("\n   Lütfen gecerli komut kodu giriniz !\n")
        return

    if ret_value == -2 :
        print("\n   TimeOut :Bootloader cevap vermiyor !")
        print("\n   Lutfen karti resetleyip tekrar deneyiniz !")
        return


def Serial_Port_Configuration(port):
    global ser
    try:
        ser = serial.Serial(port, 115200, timeout=2)
    except:
        print("\n  Girdiginiz port ismi  Gecersiz !!")

        port = serial_ports()
        if (not port):
            print("\n   Herhangi bir port tespit edilemedi!")
        else:
            print("\n   Bilgisayarınızda açık olan port listesi. Lutfen Tekrar Deneyiniz!")
            print("\n   ", port)
        return -1
    if ser.is_open:
        print("\n   Port Acık Başarılı.")
    else:
        print("\n   Port Acık Basarısız.")
    return 0


#----------------------------- MENU----------------------------------------


name = input("Lutfen baglanmak istediginiz portun ismini yaziniz(ORN: COM3):")
ret = 0
print("\n   Bu Satırdayım.")
ret=Serial_Port_Configuration(name)
if(ret < 0):
    decode_menu_command_code(0)
    

    
  
while True:
    print("\n +==========================================+")
    print(" |               Menu                       |")
    print(" |         STM32F4 BootLoader v1            |")
    print(" +==========================================+")

  
    
    print("\n Lutfen gondermek istediginiz komutu yazınız.\n")
    print("   BOOTLOADER_GET_VERSION-----------------------> 1")
    print("   BOOTLOADER_FLASH_ERASE-----------------------> 2")
    print("   BOOTLOADER_MEMORY_WRITE----------------------> 3")
    print("   MENU_EXIT------------------------------------> 0")

    command_code = input("\n   Komutu buraya giriniz :")

    if(not command_code.isdigit()):
        print("\n   Lutfen gecerli bir sayi giriniz")
    else:
        decode_menu_command_code(int(command_code))

    input("\n   Devam etmek icin herhangi bir tusa basiniz  :")
    purge_serial_port()


