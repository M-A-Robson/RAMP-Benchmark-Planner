from sparc_io import *

#run_sparc('/home/local/MTC_ORI_Collab/beam_domain_course_test.sp')
s = parse_sparc_ouput('/home/local/MTC_ORI_Collab/sparc.out')

b,a = extract_states_from_answer_set(s[0])

print(b[0], '\n')

print(b[1], '\n')

print(a)