#ifndef BR_GRAVITY_MODEL_H
#define BR_GRAVITY_MODEL_H
#define BR_G_I {{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}}
#define BR_G_RXP {{1.0f,0.0f,0.0f},{0.0f,0.0f,-1.0f},{0.0f,1.0f,0.0f}}
#define BR_G_RXN {{1.0f,0.0f,0.0f},{0.0f,0.0f,1.0f},{0.0f,-1.0f,0.0f}}
#define BR_G_LINKS { {.origin={0.0f,0.0f,0.0f},.fixed_rotation=BR_G_I,.axis={0.0f,0.0f,1.0f},.mass_kg=0.68779022f,.com_m={0.19522784f,0.01128503f,0.19430809f}}, {.origin={0.16343588f,0.05362120f,0.13859022f},.fixed_rotation=BR_G_RXN,.axis={0.0f,0.0f,1.0f},.mass_kg=0.700f,.com_m={-0.00003711f,0.21225318f,-0.04015666f}}, {.origin={0.0f,0.3635f,-0.04090258f},.fixed_rotation=BR_G_I,.axis={0.0f,0.0f,1.0f},.mass_kg=0.502f,.com_m={-0.18820938f,0.04222634f,-0.00047107f}}, {.origin={-0.25896580f,0.04980676f,0.0262f},.fixed_rotation=BR_G_RXP,.axis={0.0f,0.0f,1.0f},.mass_kg=1.008f,.com_m={-0.08065775f,0.04858973f,0.03205872f}}, {.origin={0.0f,0.0f,0.0f},.fixed_rotation=BR_G_I,.axis={0.0f,0.0f,1.0f},.mass_kg=0.0f,.com_m={0.0f,0.0f,0.0f}} }
#endif
