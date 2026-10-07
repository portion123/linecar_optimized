/* FIX4: no stop_reason/recovery fields exist; report state-derived values only. */
static void ProbeFill(uint8_t *stop,uint8_t *mode,uint8_t *attempts,uint8_t *phase,uint8_t *flags,float *aux0,float *aux1)
{
    *stop=(uint8_t)(state==CAR_LOST_STOP ? 8 : (state==CAR_ENCODER_FAULT ? 9 : 0));
    *mode=0; *attempts=0; *phase=edge_align_active; *flags=(uint8_t)(wide_active|(wide_strong<<1)|(recovering<<2));
    *aux0=(float)center_samples; *aux1=(float)lost_ms;
}
static void ProbeExtended(void) { uint16_t n=0; fwrite(&n,1,2,stdout); }
