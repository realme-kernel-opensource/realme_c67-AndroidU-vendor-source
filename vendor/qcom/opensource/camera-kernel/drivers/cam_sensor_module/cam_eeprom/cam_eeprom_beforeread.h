#ifndef EEPROM_WRITE_INIT_PARAMS_BEFORE_READ_CAL_DATA_H_
#define EEPROM_WRITE_INIT_PARAMS_BEFORE_READ_CAL_DATA_H_

#define EEPROM_MEMORY_MAP_MAX_SIZE  300
enum checkItem {
    CHECKMODULEINFO,
    CHECKAWBD50,
    CHECKAWBOVERD50,
    CHECKSN,
    CHECKLSC,
    CHECKNUMMAX,
};

enum groupNum {
    GROUPNUM0,
    GROUPNUM1,
    GROUPNUM2,
    GROUPNUMMAX,
};

struct OtpGroupInfo
{
    uint32_t    IsAvailable;
    uint32_t  CheckItemOffset[CHECKNUMMAX][GROUPNUMMAX];
    uint32_t  GroupFlag;
    uint32_t  SelectGroupNum[CHECKNUMMAX];
};

struct OtpCheckPartInfo
{
    uint32_t  IsAvailable;
    uint32_t  Offset;
    uint32_t  Length;
};

struct OplusOtpCheckInfo
{
    struct OtpGroupInfo groupInfo;
    struct OtpCheckPartInfo ItemInfo[CHECKNUMMAX][GROUPNUMMAX];
};

struct camera_reg_settings_t{
    uint32_t reg_addr;
    enum camera_sensor_i2c_type addr_type;
    uint32_t reg_data;
    enum camera_sensor_i2c_type data_type;
    uint32_t delay;
};

struct eeprom_memory_map_init_write_params{
    uint32_t slave_addr;
    struct camera_reg_settings_t mem_settings[EEPROM_MEMORY_MAP_MAX_SIZE];
    uint32_t memory_map_size;
};

struct eeprom_memory_map_init_write_params hi846w_eeprom  ={
    .slave_addr = 0x40,
          .mem_settings =
          {
            {0x0A00, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0000, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x2000, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0000, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x2002, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00FF, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x2004, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0000, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x2008, CAMERA_SENSOR_I2C_TYPE_WORD, 0x3FFF, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x23FE, CAMERA_SENSOR_I2C_TYPE_WORD, 0xC056, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0A00, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0000, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0E04, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0012, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F08, CAMERA_SENSOR_I2C_TYPE_WORD, 0x2F04, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F30, CAMERA_SENSOR_I2C_TYPE_WORD, 0x001F, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F36, CAMERA_SENSOR_I2C_TYPE_WORD, 0x001F, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F04, CAMERA_SENSOR_I2C_TYPE_WORD, 0x3A00, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F32, CAMERA_SENSOR_I2C_TYPE_WORD, 0x025A, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F38, CAMERA_SENSOR_I2C_TYPE_WORD, 0x025A, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x0F2A, CAMERA_SENSOR_I2C_TYPE_WORD, 0x4124, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x006A, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0100, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},
            {0x004C, CAMERA_SENSOR_I2C_TYPE_WORD, 0x0100, CAMERA_SENSOR_I2C_TYPE_WORD, 0x00},

            {0x0a02, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x0a00, CAMERA_SENSOR_I2C_TYPE_WORD,0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 10},
            {0x0f02, CAMERA_SENSOR_I2C_TYPE_WORD,0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x071a, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x071b, CAMERA_SENSOR_I2C_TYPE_WORD,0x09, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x0d04, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x0d00, CAMERA_SENSOR_I2C_TYPE_WORD,0x07, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x003e, CAMERA_SENSOR_I2C_TYPE_WORD,0x10, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x0a00, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 1 },

            {0x070a, CAMERA_SENSOR_I2C_TYPE_WORD,0x02, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x070b, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x0702, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },


          },
          .memory_map_size = 29,
};
struct eeprom_memory_map_init_write_params hi846w_eeprom_after_read  ={
    .slave_addr = 0x40,
          .mem_settings =
          {
            {0x0a00, CAMERA_SENSOR_I2C_TYPE_WORD,0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 10},
            {0x003e, CAMERA_SENSOR_I2C_TYPE_WORD,0x00, CAMERA_SENSOR_I2C_TYPE_BYTE, 0 },
            {0x0a00, CAMERA_SENSOR_I2C_TYPE_WORD,0x01, CAMERA_SENSOR_I2C_TYPE_BYTE, 1 },
          },
          .memory_map_size = 3,
};

struct  OplusOtpCheckInfo hi846w_otp_checkinfo ={
    .groupInfo =
    {
        .IsAvailable = TRUE,
        .GroupFlag   = 0x01,
        .CheckItemOffset =
        {
            {0x000F, 0x073c, 0x0E69},
            {0x0021, 0x074E, 0x0E7B},
            {0x0027, 0x0754, 0x0E81},
            {0x0041, 0x076E, 0x0E9B},
            {0x072B, 0x0E58, 0x1585},
        },
    },

    .ItemInfo =
    {
        { { TRUE, 0x0000, 0x0011 }, { TRUE, 0x072D, 0x0011 }, { TRUE, 0x0E5A, 0x0011 }, },
        { { TRUE, 0x0011, 0x0012 }, { TRUE, 0x073E, 0x0012 }, { TRUE, 0x0E6B, 0x0012 }, },
        { { TRUE, 0x0023, 0x0006 }, { TRUE, 0x0750, 0x0006 }, { TRUE, 0x0E7D, 0x0006 }, },
        { { TRUE, 0x0029, 0x001A }, { TRUE, 0x0756, 0x001A }, { TRUE, 0x0E83, 0x001A }, },
        { { TRUE, 0x0043, 0x06E8 }, { TRUE, 0x0770, 0x06E8 }, { TRUE, 0x0E9D, 0x06E8 }, },
    },
};

int eeprom_memory_map_read_data(uint32_t slave_addr,struct cam_eeprom_memory_map_t emap,
                                struct cam_eeprom_ctrl_t *e_ctrl,uint8_t *memptr);

int eeprom_process_para_before_read(uint32_t slave_addr,
                                    struct eeprom_memory_map_init_write_params **pWriteParams,
                                    struct OplusOtpCheckInfo **pOtpParams,uint32_t *count_read);
int eeprom_memory_select_group(struct OplusOtpCheckInfo *pOtpParams);
int eeprom_memory_process_otp_data(struct OplusOtpCheckInfo *pOtpParams,uint8_t *memptr);
int eeprom_memory_map_unint_para(uint32_t slave_addr,struct cam_eeprom_ctrl_t *e_ctrl);
int eeprom_process_para_after_read(uint32_t slave_addr,struct eeprom_memory_map_init_write_params **pWriteParams);
#endif
