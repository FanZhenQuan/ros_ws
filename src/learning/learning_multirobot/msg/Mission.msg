string name
int32 mission_id
int32 robot_id
float32 action
# action puo' rappresentare:
# @ 1 | -1 costante moltiplicativa per andare avanti | indietro     quando name=="walk"
# @ un angolo di rotazione, compreso tra -180 e 180                 quando name=="rotate"
# @ -inf                                                            quando name=="report"