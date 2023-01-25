from sparc_io import run_sparc
import time

t1 = time.perf_counter()
run_sparc('/home/local/MTC_ORI_Collab/sparc_planning/sparc_files/beam_domain_coarse.sp')
t2 = time.perf_counter()
print(f'time taken : {t2 - t1} seconds')
