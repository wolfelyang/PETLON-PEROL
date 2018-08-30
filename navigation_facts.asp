#program base.

room(l3_414b).
room(l3_414a).
room(l3_402).
room(l3_520).
room(l3_400).
room(l3_508).
room(l3_428).
room(l3_404).
room(l3_424).
room(l3_502).
room(l3_426).
room(l3_500).
room(l3_420).
room(l3_506).
room(l3_422).
room(l3_504).
room(l3_200).
room(l3_300).
room(l3_303).
room(l3_302).
room(l3_406).
room(l3_250).
room(l3_410).
room(l3_412).
room(l3_518).
room(l3_414).
room(l3_416).
room(l3_514).
room(l3_418).
room(l3_516).
room(l3_430).
room(l3_510).
room(l3_436).
room(l3_512).
room(l3_434).
room(l3_432).
room(l3_408).

room(l3_828).
room(l3_824).
room(l3_818).
room(l3_816).
room(l3_814).
room(l3_830).
room(l3_728).
room(l3_724).
room(l3_722).
room(l3_710b).
room(l3_710a).
room(l3_710).
room(l3_804).
room(l3_802).
room(l3_718).
room(l3_702).
room(l3_700).
room(l3_800).
room(l3_600).
room(l3_100).



door(d3_404).
door(d3_400).
door(d3_508).
door(d3_402).
door(d3_500).
door(d3_502).
door(d3_430).
door(d3_422).
door(d3_420).
door(d3_414a2).
door(d3_414a3).
door(d3_414a1).
door(d3_416).
door(d3_516).
door(d3_418).
door(d3_512).
door(d3_510).
door(d3_414b3).
door(d3_414b2).
door(d3_414b1).
door(d3_432).
door(d3_436).

door(d3_824).
door(d3_816a).
door(d3_710b1).
door(d3_710b2).
door(d3_710b3).
door(d3_710a1).
door(d3_710a2).
door(d3_710a3).
door(d3_600).
door(d3_303).



hasdoor(l3_404,d3_404).
hasdoor(l3_400,d3_404).

hasdoor(l3_400,d3_400).
hasdoor(l3_300,d3_400).

hasdoor(l3_500,d3_508).
hasdoor(l3_508,d3_508).

hasdoor(l3_402,d3_402).
hasdoor(l3_400,d3_402).

hasdoor(l3_500,d3_500).
hasdoor(l3_200,d3_500).

hasdoor(l3_500,d3_502).
hasdoor(l3_502,d3_502).

hasdoor(l3_430,d3_430).
hasdoor(l3_400,d3_430).

hasdoor(l3_422,d3_422).
hasdoor(l3_400,d3_422).

hasdoor(l3_420,d3_420).
hasdoor(l3_400,d3_420).

%please be coherent with the simulation domain with a1 and a2
hasdoor(l3_500,d3_414a2).
hasdoor(l3_414a,d3_414a2).

hasdoor(l3_414,d3_414a3).
hasdoor(l3_414a,d3_414a3).

%please be coherent with the simulation domain with a1 and a2
hasdoor(l3_414a,d3_414a1).
hasdoor(l3_400,d3_414a1).

hasdoor(l3_416,d3_416).
hasdoor(l3_400,d3_416).

hasdoor(l3_500,d3_516).
hasdoor(l3_516,d3_516).

hasdoor(l3_418,d3_418).
hasdoor(l3_400,d3_418).

hasdoor(l3_500,d3_512).
hasdoor(l3_512,d3_512).

hasdoor(l3_500,d3_510).
hasdoor(l3_510,d3_510).

hasdoor(l3_414b,d3_414b3).
hasdoor(l3_414,d3_414b3).

hasdoor(l3_400,d3_414b2).
hasdoor(l3_414b,d3_414b2).

hasdoor(l3_414b,d3_414b1).
hasdoor(l3_500,d3_414b1).

hasdoor(l3_432,d3_432).
hasdoor(l3_400,d3_432).

hasdoor(l3_436,d3_436).
hasdoor(l3_400,d3_436).

hasdoor(l3_824,d3_824).
hasdoor(l3_800,d3_824).

%hasdoor(l3_816,d3_816a).
%hasdoor(l3_814,d3_816a).

hasdoor(l3_816,d3_816a).
hasdoor(l3_800,d3_816a).

hasdoor(l3_710b,d3_710b1).
hasdoor(l3_800,d3_710b1).

hasdoor(l3_710b,d3_710b2).
hasdoor(l3_700,d3_710b2).

hasdoor(l3_710b,d3_710b3).
hasdoor(l3_710,d3_710b3).

hasdoor(l3_710a,d3_710a1).
hasdoor(l3_700,d3_710a1).

hasdoor(l3_710a,d3_710a2).
hasdoor(l3_800,d3_710a2).

hasdoor(l3_710a,d3_710a3).
hasdoor(l3_710,d3_710a3).

hasdoor(l3_600,d3_600).
hasdoor(l3_100,d3_600).

hasdoor(l3_303,d3_303).
hasdoor(l3_100,d3_303).



acc(l3_434, l3_400).
acc(l3_434, l3_500).
acc(l3_518, l3_500).
acc(l3_514, l3_500).
acc(l3_504, l3_500).
acc(l3_520, l3_500).
acc(l3_410, l3_500).
acc(l3_424, l3_400).
acc(l3_408, l3_400).
acc(l3_410, l3_400).
acc(l3_200, l3_303).
acc(l3_302, l3_303).
acc(l3_302, l3_300).
acc(l3_250, l3_300).
acc(l3_250, l3_303).

acc(l3_406, l3_400).

acc(l3_828, l3_800).
acc(l3_818, l3_800).
acc(l3_814, l3_800).
acc(l3_830, l3_800).
acc(l3_728, l3_800).
acc(l3_728, l3_700).
acc(l3_724, l3_800).
acc(l3_724, l3_700).
acc(l3_722, l3_700).
acc(l3_718, l3_700).
acc(l3_600, l3_700).
acc(l3_702, l3_700).
acc(l3_804, l3_800).
acc(l3_804, l3_700).
acc(l3_802, l3_800).
acc(l3_802, l3_700).


dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), R1 != R2, door(D), room(R1), room(R2).
dooracc(R1,D,R2) :- dooracc(R2,D,R1).

acc(R1,R1) :- room(R1).
acc(R1,R2) :- acc(R2,R1), room(R1), room(R2).
acc(R1,R2) :- acc(R1,R3), acc(R2,R3), room(R1), room(R2), room(R3).
