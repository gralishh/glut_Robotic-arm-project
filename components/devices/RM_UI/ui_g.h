//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[0];
extern ui_interface_string_t ui_g_now_strings[1];
extern uint8_t ui_g_dirty_figure[0];
extern uint8_t ui_g_dirty_string[1];


#define ui_g_UwUGrp_UwUText (&(ui_g_now_strings[0]))

#ifdef MANUAL_DIRTY

#define ui_g_UwUGrp_UwUText_dirty (ui_g_dirty_string[0])
#endif

void ui_init_g();
void ui_update_g();

#endif //UI_g_H
