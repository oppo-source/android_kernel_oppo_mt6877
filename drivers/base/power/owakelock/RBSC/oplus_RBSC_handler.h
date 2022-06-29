/***********************************************************
** Copyright (C), 2008-2019, OPLUS Mobile Comm Corp., Ltd.
****************************************************************/

int oplus_save_clk_regulator_info_start(void);
void oplus_save_clk_regulator_info_end(void);
void oplus_get_clk_regulater_info(void);
void write_buff_to_dumpfile(char* buff);
void RBSC_issue_handler(int mode, char* issue_type);
int is_clk_dump_start_record(void);