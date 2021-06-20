%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%  GENERAL DEFINITION  %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% NODE %%%
state(Nid,pair(X,Y)):- init(object(node,Nid),value(at,pair(X,Y))).
node(Nid):- init(object(node,Nid),value(at,pair(X,Y))).
node_Num(N):- N=#count{Nid:init(object(node,Nid),value(at,pair(X,Y)))}.


%%% GRID %%%
state(object(highway,Nid)):- init(object(highway,Nid),value(at,pair(X,Y))).
num_Col(num_cols):- num_cols = #count{X:init(object(node,Nid),value(at,pair(X,Y)))}.
num_Row(num_rows):- num_rows = #count{Y:init(object(node,Nid),value(at,pair(X,Y)))}.


%%% ROBOT %%%
state(object(robot,Rid),on(node,Nid),0):- state(Nid,pair(X,Y)), init(object(robot,Rid),value(at,pair(X,Y))).

robot(Rid):- init(object(robot,Rid),value(at,pair(X,Y))).

move(1,0;-1,0;0,1;0,-1).

numRobots(Num):- Num = #count{Rid:init(object(robot,Rid),value(at,pair(X,Y)))}.

{occurs(object(robot,Rid),move(X1,Y1),T):move(X1,Y1)}1:- numRobots(Num), Rid=1..Num, T=0..n-1.

state(object(robot,Rid),on(node, Moved_Nid), T+1):- state(Nid,pair(X,Y)), 
state(object(robot,Rid),on(node,Nid),T), occurs(object(robot,Rid),move(X1,Y1),T), state(Moved_Nid, pair(X+X1,Y+Y1)).


%%% ORDER %%
state(Oid,object(node,Nid),product_info(Pid,PQ),0):- init(object(order,Oid),value(pickingStation,PKid)), state(object(pickingStation,PKid),object(node,Nid)), init(object(order,Oid),value(line,pair(Pid,PQ))).


%%% PICKING STATION %%%
state(object(pickingStation,PKid),object(node,Nid)) :- init(object(pickingStation,PKid),value(at,pair(X,Y))), init(object(node,Nid),value(at,pair(X,Y))).


%%% SHELF %%%
shelf(Sid):- init(object(shelf,Sid),value(at,pair(X,Y))).
state(object(shelf,Sid),on(node,Nid),0) :- init(object(shelf,Sid),value(at,pair(X,Y))), state(Nid,pair(X,Y)).


%%% PRODUCT %%%
product(Pid):- init(object(product,Pid),value(on,pair(Sid,EA))).
state(object(product,Pid),object(shelf,Sid),with(quantity,EA),0):- init(object(product,Pid),value(on,pair(Sid,EA))).
numProducts(Num):- Num=#count{Pid:init(object(product,Pid),value(on,pair(X,Y)))}.


%%% PICKUP %%%
% Need to specifiy the shelf
{occurs(pickup(Rid,Sid),T):shelf(Sid)}1:- Rid=1..NR, numRobots(NR), T=0..n-1.
occurs(object(robot,Rid),pickup,T):- occurs(pickup(Rid,_),T).
% Effect of picking up a shelf
state(object(shelf,Sid),on(robot,Rid),T+1) :- occurs(object(robot,Rid),pickup,T), state(object(shelf,Sid),on(node,Nid),T), state(object(robot,Rid),on(node,Nid),T), R=1..NR, numRobots(NR).


%%% PUTDOWN %%%
{occurs(putdown(Rid,Sid),T):shelf(Sid)}1:- Rid=1..NR, numRobots(NR), T=0..TN,TN=n-1.
occurs(object(robot,Rid),putdown,T):-occurs(putdown(Rid,_),T).
% Effect of putting down a shelf
state(object(shelf,Sid),on(node,Nid),T+1) :- occurs(putdown(Rid,Sid),T), state(object(shelf,Sid),on(robot,Rid),T), state(object(robot,Rid),on(node,Nid),T).


%%% DELIVERY %%%
{occurs(object(robot,Rid),for(order,Oid),deliver(Sid,Pid,EA),T):state(Oid,object(node,Nid),product_info(Pid,EA),T), 
state(object(product,Pid),object(shelf,Sid),with(quantity,PQ),T), EA=1..PQ}1:- Rid=1..NR, numRobots(NR), T=0..TN,TN=n-1.
occurs(object(robot,Rid),deliver(Oid,Pid,EA),T):-occurs(object(robot,Rid),for(order,Oid),deliver(Sid,Pid,EA),T).
state(Oid,object(node,Nid),product_info(Pid,EA-QQ),T+1):- occurs(object(robot,Rid),for(order,Oid),deliver(Sid,Pid,QQ),T), 
state(Oid,object(node,Nid),product_info(Pid,EA),T).
state(object(product,Pid),object(shelf,Sid),with(quantity,PQ-QQ),T+1):- occurs(object(robot,R),for(order,OI),deliver(Sid,Pid,QQ),T), 
state(object(product,Pid),object(shelf,Sid),with(quantity,PQ),T).


%%%%%%%%%%%%%%%%%%%%%  GOAL  %%%%%%%%%%%%%%%%%%%%%
:- not state(Oid,object(node,Nid),product_info(Pid,0),T), state(Oid,object(node,Nid),product_info(Pid,_),0), 
Nid=1..NN, node_Num(NN), T=n.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%  LAW OF INERTIA  %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state(object(robot,Rid),on(node,Nid),T+1):- state(object(robot,Rid),on(node,Nid),T), not occurs(object(robot,Rid),move(_,_),T), T<n.
state(Oid,object(node,Nid),product_info(Pid,OQ),T+1):- state(Oid,object(node,Nid),product_info(Pid,OQ),T), 
state(object(product,Pid),object(shelf,Sid),with(quantity,PQ),T), not occurs(object(robot,_),for(order,Oid),deliver(Sid,Pid,_),T), T<n.
state(object(product,Pid),object(shelf,Sid),with(quantity,PQ),T+1):- state(object(product,Pid),object(shelf,Sid),with(quantity,PQ),T), 
not occurs(object(robot,_),for(order,_),deliver(Sid,Pid,_),T), T<n.
state(object(shelf,Sid),on(robot,Rid),T+1):-state(object(shelf,Sid),on(robot,Rid),T), not occurs(putdown(Rid,Sid),T), T<n.
state(object(shelf,Sid),on(node,Nid),T+1):-state(object(shelf,Sid),on(node,Nid),T), not occurs(pickup(_,Sid),T), T<n.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%  CONSTRAINTS  %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%  ROBOT  %%%%%%%%%%%%%%%%%%%%%
% Cannot move to more than 1 slot apart
:- not move(1,0;-1,0;0,1;0,-1).
:- occurs(object(robot,R),move(DX,DY),T), DX>1, DX<-1, DY>1, DY<-1.
% Within the range of grid
:- occurs(object(robot,Rid),move(X1,Y1),T), state(object(robot,Rid),on(node,Nid),T), state(Nid,pair(X,Y)), X+X1>NC, num_Col(NC).
:- occurs(object(robot,Rid),move(X1,Y1),T), state(object(robot,Rid),on(node,Nid),T), state(Nid,pair(X,Y)), Y+Y1>NR, num_Row(NR).
:- occurs(object(robot,Rid),move(X1,Y1),T), state(object(robot,Rid),on(node,Nid),T), state(Nid,pair(X,Y)), X+X1<1.
:- occurs(object(robot,Rid),move(X1,Y1),T), state(object(robot,Rid),on(node,Nid),T), state(Nid,pair(X,Y)), Y+Y1<1.
% Cannot do pickup/putdown/deliver and move concurrently
:- occurs(object(robot,Rid),move(X1,Y1),T), occurs(pickup(Rid,_),T).
:- occurs(object(robot,Rid),move(X1,Y1),T), occurs(putdown(Rid,_),T).
:- occurs(object(robot,Rid),move(X1,Y1),T), occurs(object(robot,Rid),for(order,_),deliver(_,_,_),T).
% Cannot move under other shelves if a robot holds a shelf already
:- state(object(robot,Rid),on(node,Nid),T), occurs(object(robot,Rid),move(DX,DY),T), state(Nid,pair(X,Y)), state(object(shelf,Sid),on(robot,Rid),T), 
state(object(shelf,Sid_other),on(node,Nid_other),T), state(Nid_other,pair(X+DX,Y+DY)), Sid!=Sid_other, Nid!=Nid_other.
% No two actions concurrently
:- occurs(object(robot,Rid),Move,T), occurs(object(robot,Rid),Move_other,T), Move!=Move_other.


%%%%%%%%%%%%%%%%%%%%%  PICKUP  %%%%%%%%%%%%%%%%%%%%%
% No 2 shelves on the same location
:- state(object(shelf,Sid),on(node,Nid),T), state(object(shelf,Sid2),on(node,Nid),T), Sid!=Sid2.
:- state(object(shelf,Sid),on(robot,Rid),T), state(object(shelf,Sid2),on(robot,Rid),T), Sid!=Sid2.
% Cannot pick up the shelve more than 1
:- occurs(pickup(Rid,Sid),T), state(object(shelf,Sid_other),on(robot,Rid),T), Sid!=Sid_other.
% Cannot take away the shelf from other robot
:- occurs(pickup(Rid,Sid),T), state(object(shelf,Sid),on(robot,Rid_other),T), Rid!=Rid_other.
% No 2 robots pick up the same shelf
:- 2{occurs(pickup(Rid,Sid),T): robot(Rid)}, shelf(Sid).
% Can pick a shelf only on the node where the shelf reside
:- occurs(pickup(Rid,Sid),T), state(object(shelf,Sid),on(node,Nid),T), not state(object(robot,Rid),on(node,Nid),T). 


%%%%%%%%%%%%%%%%%%%%%  PUTDOWN  %%%%%%%%%%%%%%%%%%%%%
% Robot can putdown only if they holds a shelf.
:- occurs(putdown(Rid,Sid),T), not state(object(shelf,Sid),on(robot,Rid),T).
% No putdown a shelf on the highway
:- occurs(putdown(Rid,Sid),T), state(object(robot,Rid),on(node,Nid),T), state(object(highway,Nid)). 


%%%%%%%%%%%%%%%%%%%%%  DELIVERY  %%%%%%%%%%%%%%%%%%%%%
% Robot can do deliver only when it holds a shelves containing product
:- occurs(object(robot,Rid),for(order,Oid),deliver(Sid,Pid,_),T), state(object(product,Pid),object(shelf,Sid),with(quantity,_),T), not state(object(shelf,Sid),on(robot,Rid),T).
% Robot can do deliver only when it is at corresponding pickingstation
:- occurs(object(robot,Rid),for(order,Oid),deliver(_,Pid,_),T), state(Oid,object(node,Nid),product_info(Pid,_),T), not state(object(robot,Rid),on(node, Nid),T), 
state(object(pickingStation,PKid),object(node,Nid)).
% Robot cannot do deliver more than product amount
:- occurs(object(robot,Rid),for(order,Oid),deliver(Sid,Pid,QQ),T), state(object(product,Pid),object(shelf,Sid),with(quantity,PQ),T), QQ>PQ.
% Robot cannot do deliver if order amount is larger than product amount
:- occurs(object(robot,Rid),for(order,Oid),deliver(Sid,Pid,QQ),T), state(Oid,object(node,Nid),product_info(Pid,PQ),T), QQ>PQ.


%%%%%%%%%%%%%%%%%%    ROBOT STATES     %%%%%%%%%%%%%%%%%%
% Cannot swap the location = no collision
:- state(object(robot,Rid1),on(node,Nid),T), state(object(robot,Rid1),on(node,Nid2),T+1), state(object(robot,Rid2),on(node,Nid2),T), state(object(robot,Rid2),on(node,Nid),T+1), Rid1!=Rid2.
% No 2 robots are on same location
:- 2{state(object(robot,Rid),on(node,Nid),T):node(Nid)}, robot(Rid), T=0..n.
:- 2{state(object(robot,Rid),on(node,Nid),T):robot(Rid)}, node(Nid), T=0..n.


%%%%%%%%%%%%%%%%%%    SHELF STATES     %%%%%%%%%%%%%%%%%%
% Shlef cannot be at both robot and node concurrently
:- state(object(shelf,Sid),on(node,_),T), state(object(shelf,Sid),on(robot,_),T).
:- state(object(shelf,Sid),on(node,Nid),T), state(object(shelf,Sid),on(node,Nid2),T), Nid!=Nid2.
:- state(object(shelf,Sid),on(robot,Rid),T), state(object(shelf,Sid),on(robot,Rid2),T), Rid!=Rid2.
timestamp(N):-N=#count{T:occurs(A,B,T)}.
#minimize{N: timestamp(N)}.
#show occurs/3.
