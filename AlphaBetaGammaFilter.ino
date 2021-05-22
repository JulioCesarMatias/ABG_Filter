/*
  AUTOR:QUICK-FLASH - EMUFLIGHT
  https://github.com/emuflight/EmuFlight/blob/master/src/main/common/filter.c

  BASEADO EM:
  http://yadda.icm.edu.pl/yadda/element/bwmeta1.element.baztech-922ff6cb-e991-417f-93f0-77448f1ef4ec/c/A_Study_Jeong_1_2017.pdf
*/

typedef struct PT1Filter
{
  float State;
  float K;
} PT1_Filter_Struct;

typedef struct
{
  float a, b, g, e;
  float aK, vK, xK, jK;
  float DeltaTime, DeltaTime2, DeltaTime3;
  float HalfLife, Boost;
  PT1_Filter_Struct VelocityFilter, AcelerationFilter, JerkFilter;
} AlphaBetaGammaFilter_Struct;

void PT1FilterInitialization(PT1_Filter_Struct *Filter_Pointer, float K) {
  Filter_Pointer->State = 0.0f;
  Filter_Pointer->K = K;
}

float PT1FilterCalculeGain(uint16_t CutOffFrequency, float DeltaTime) {
  const float RC = 0.5f / (3.1415926535897932384626433832795f * CutOffFrequency);
  return DeltaTime / (RC + DeltaTime);
}

float PT1FilterApply(PT1_Filter_Struct *Filter_Pointer, float Input) {
  Filter_Pointer->State = Filter_Pointer->State + Filter_Pointer->K * (Input - Filter_Pointer->State);
  return Filter_Pointer->State;
}

void ABG_Initialization(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Alpha, int16_t BoostGain, int16_t HalfLife, float DeltaTime)
{
  const float Alpha2 = Alpha * 0.001f;
  const float xi = powf(-Alpha2 + 1.0f, 0.25);
  Filter_Pointer->xK = 0.0f;
  Filter_Pointer->vK = 0.0f;
  Filter_Pointer->aK = 0.0f;
  Filter_Pointer->jK = 0.0f;
  Filter_Pointer->a = Alpha2;
  Filter_Pointer->b = (1.0f / 6.0f) * powf(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
  Filter_Pointer->g = 2 * powf(1.0f - xi, 3) * (1 + xi);
  Filter_Pointer->e = (1.0f / 6.0f) * powf(1 - xi, 4);
  Filter_Pointer->DeltaTime = DeltaTime;
  Filter_Pointer->DeltaTime2 = DeltaTime * DeltaTime;
  Filter_Pointer->DeltaTime3 = DeltaTime * DeltaTime * DeltaTime;

  PT1FilterInitialization(&Filter_Pointer->VelocityFilter, PT1FilterCalculeGain(75, DeltaTime));
  PT1FilterInitialization(&Filter_Pointer->AcelerationFilter, PT1FilterCalculeGain(50, DeltaTime));
  PT1FilterInitialization(&Filter_Pointer->JerkFilter, PT1FilterCalculeGain(25, DeltaTime));

  Filter_Pointer->Boost = (BoostGain * BoostGain / 1000000) * 0.003;
  Filter_Pointer->HalfLife = HalfLife != 0 ? powf(0.5f, DeltaTime / HalfLife / 100.0f) : 1.0f;
}

float AlphaBetaGammaApply(AlphaBetaGammaFilter_Struct *Filter_Pointer, float Input)
{
  float rK; //ERRO RESIDUAL

  Filter_Pointer->xK *= Filter_Pointer->HalfLife;
  Filter_Pointer->vK *= Filter_Pointer->HalfLife;
  Filter_Pointer->aK *= Filter_Pointer->HalfLife;
  Filter_Pointer->jK *= Filter_Pointer->HalfLife;

  //ATUALIZA O ESTADO ESTIMADO DO SISTEMA
  Filter_Pointer->xK += Filter_Pointer->DeltaTime * Filter_Pointer->vK + (1.0f / 2.0f) * Filter_Pointer->DeltaTime2 * Filter_Pointer->aK + (1.0f / 6.0f) * Filter_Pointer->DeltaTime3 * Filter_Pointer->jK;

  //ATUALIZA A VELOCIDADE ESTIMADA
  Filter_Pointer->vK += Filter_Pointer->DeltaTime * Filter_Pointer->aK + 0.5f * Filter_Pointer->DeltaTime2 * Filter_Pointer->jK;
  Filter_Pointer->aK += Filter_Pointer->DeltaTime * Filter_Pointer->jK;

  //CALCULA O ERRO RESIDUAL
  rK = Input - Filter_Pointer->xK;

  //AUMENTA ARTIFICIALMENTE O ERRO PARA AUMENTAR A RESPOSTA DO FILTRO
  rK += (fabsf(rK) * rK * Filter_Pointer->Boost);

  //ATUALIZA A ESTIMATIVA DE ERRO RESIDUAL
  Filter_Pointer->xK += Filter_Pointer->a * rK;
  Filter_Pointer->vK += Filter_Pointer->b / Filter_Pointer->DeltaTime * rK;
  Filter_Pointer->aK += Filter_Pointer->g / (2.0f * Filter_Pointer->DeltaTime2) * rK;
  Filter_Pointer->jK += Filter_Pointer->e / (6.0f * Filter_Pointer->DeltaTime3) * rK;

  Filter_Pointer->vK = PT1FilterApply(&Filter_Pointer->VelocityFilter, Filter_Pointer->vK);
  Filter_Pointer->aK = PT1FilterApply(&Filter_Pointer->AcelerationFilter, Filter_Pointer->aK);
  Filter_Pointer->jK = PT1FilterApply(&Filter_Pointer->JerkFilter, Filter_Pointer->jK);

  return Filter_Pointer->xK;
}

AlphaBetaGammaFilter_Struct ABGFilter_Test;

#define THIS_LOOP_RATE 1000 //1KHZ

const float AlphaDefault = 10; //VALOR PEQUENO
const int16_t BoostDefault = 275;
const int16_t HalfLifeDefault = 50;

void setup() {
  Serial.begin(115200);
  ABG_Initialization(&ABGFilter_Test, AlphaDefault, BoostDefault, HalfLifeDefault, THIS_LOOP_RATE * 1e-6f);
}

void loop() {
  int16_t GetAnalogRead = analogRead(0);
  float ABG_AnalogRead =  AlphaBetaGammaApply(&ABGFilter_Test, GetAnalogRead);
  Serial.print(GetAnalogRead);
  Serial.print("   ");
  Serial.println(ABG_AnalogRead);
  delay(1); //1KHZ LOOP
}
