/* work firmware observation for sim/server (read-only access to Track.c statics). */
static void ProbeFill(uint8_t *stop,uint8_t *mode,uint8_t *attempts,uint8_t *phase,uint8_t *flags,float *aux0,float *aux1)
{
    *stop=(uint8_t)stop_reason; *mode=0; *attempts=recovery_attempts; *phase=edge_align_active;
    *flags=(uint8_t)(wide_active|(wide_strong<<1)|(recovering<<2)|(recovery_active<<3));
    *aux0=recovery_angle; *aux1=recovery_distance;
}
static void ProbeExtended(void) { uint16_t n=0; fwrite(&n,1,2,stdout); }
