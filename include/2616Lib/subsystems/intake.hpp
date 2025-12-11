#pragma once

void set_intake(int voltage);
void intake_timed(int voltage, long millis);
void control_intake();
void set_top_roller(int voltage);
void set_botom_roller(int voltage);
void set_back_roller(int voltage);
void set_rollers(int voltage);
void score_mid(int voltage);
void control_rollers();
void store_top();
void score_from_hold();
void score_bottom(int voltage);