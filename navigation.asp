#include <incmode>.

#const istop  = "SAT".
#const imin   = 1.
#const imax  = 50.

#program base.
cost(0,0).

heuristics(R1,approach(D),10) :- hasdoor(R1,D).
heuristics(R1,gothrough(D),10) :- hasdoor(R1,D).


#program step(k).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{approach(D,k)} :- at(R1,k), hasdoor(R2,D), acc(R1,R2).

facing(D,k) :- approach(D,k-1).
beside(D,k) :- approach(D,k-1).
at(R2,k) :- approach(D,k-1), at(R1,k-1), hasdoor(R2,D), acc(R1,R2).
:- approach(D,k-1), at(R1,k-1), dooracc(R3,D,R2), not acc(R1,R3), not acc(R1,R2).

%{gothrough(D,k)} :- facing(D,k),open(D,k),beside(D,k).
at(R2,k) :- gothrough(D,k-1), at(R1,k-1), dooracc(R1,D,R2).
-facing(D,k) :- gothrough(D,k-1), facing(D,k-1).
:- gothrough(D,k-1), not facing(D,k-1).
:- gothrough(D,k-1), not open(D,k-1).
%cannot go through if the person in office is not available
:- gothrough(D,k-1), at(R1,k-1), dooracc(R1,D,R2), hasoffice(P,R2), not accessgranted(D,k-1).

%{open(D,k)} :- facing(D,k),-open(D,k),beside(D,k).
open(D,k) :- opendoor(D,k-1).
facing(D,k) :- opendoor(D,k-1).
beside(D,k) :- opendoor(D,k-1).
:- opendoor(D,k-1), badDoor(D).
:- opendoor(D,k-1), not facing(D,k-1).
:- opendoor(D,k-1), closedofficedoor(D,k-1).
:- opendoor(D,k-1), open(D,k-1).

facing(O,k) :- goto(O,k-1).
beside(O,k) :- goto(O,k-1).
at(R,k) :- goto(O,k-1), inside(O,R).
:- goto(O,k-1), inside(O,R1), at(R2,k-1), not acc(R1,R2).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Change on plan quality
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cost(Q+R,k) :- approach(D,k-1), at(R1,k-1), cost(Q,k-1), ro(at(R1),approach(D),R), #count{D1:facing(D1)}=0, #count{D1:beside(D1)}=0.
cost(Q+R,k) :- approach(D,k-1), at(R1,k-1), cost(Q,k-1), ro((at(R1),facing(D1)),approach(D),R), facing(D1).
cost(Q+R,k) :- approach(D,k-1), at(R1,k-1), cost(Q,k-1), ro((at(R1),beside(D1)),approach(D),R), beside(D1).
% if there is learned value, use it

cost(Q+10,k) :- approach(D,k-1), at(R1,k-1), cost(Q,k-1), #count{R:ro((at(R1),facing(D1)),approach(D),R)}=0,
                                                          #count{R:ro(at(R1),approach(D),R)}=0,
                                                          #count{R:ro((at(R1),beside(D1)),approach(D),R)}=0.
                                                          
% if there is no learned value, use heuristics obtained from navigation

cost(Q+R,k) :- gothrough(D,k-1), at(R1,k-1), dooracc(R1,D,R2), cost(Q,k-1), ro((at(R1),beside(D),facing(D)),gothrough(D),R).
cost(Q+10,k) :- gothrough(D,k-1), at(R1,k-1), dooracc(R1,D,R2), cost(Q,k-1), #count{R:ro((at(R1),beside(D),facing(D)),gothrough(D),R)}=0.

cost(Q+R,k) :- opendoor(D,k-1), at(R1,k-1), dooracc(R1,D,R2), cost(Q,k-1), ro((at(R1),beside(D),facing(D)),opendoor(D),R).
cost(Q+10,k) :- opendoor(D,k-1), at(R1,k-1), dooracc(R1,D,R2), cost(Q,k-1), #count{R:ro((at(R1),beside(D),facing(D)),opendoor(D),R)}=0.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%you can't be at two places at the some time
-at(L2,k):- at(L1,k), room(L2), L1 != L2.

%you can only be beside a door at any given time (if you delete this,
%the observations must also return -beside which doesn't happen at the moment.
-beside(D,k) :- beside(D,k-1), at(R,k), not hasdoor(R,D), door(D).
-beside(O2,k):- beside(O1,k), beside(O2,k-1), O1 != O2.
-beside(O,k) :- beside(O,k-1), at(R,k), not inside(O,R), object(O).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cost is inertial
{cost(R,k)} :- cost(R,k-1).
:- #count{Q:cost(Q,k)}!=1.

%at is inertial
at(L,k) :- at(L,k-1), not -at(L,k).
%facing is inertial
facing(D,k) :- facing(D,k-1), not -facing(D,k).
-facing(D,k) :- -facing(D,k-1), not facing(D,k).
%you can be facing only one door at a time
-facing(O2,k):- facing(O1,k), facing(O2,k-1), O1 != O2.
% open is inertial
open(D,k) :- open(D,k-1), not -open(D,k).
-open(D,k) :- -open(D,k-1), not open(D,k), not -knowclosed(D,k).
% beside is inertial
beside(D,k) :- beside(D,k-1), not -beside(D,k).


#show at/2.
#show open/2.
#show -open/2.
#show facing/2.
#show beside/2.

#show approach/2.
#show gothrough/2.
#show opendoor/2.
#show goto/2.
#show callelevator/3.
#show changefloor/2.
#show knock/2.
#show searchperson/4.
#show delivermessage/3.
#show cost/2.
