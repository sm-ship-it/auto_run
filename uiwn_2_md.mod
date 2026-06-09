/* Given parameters */
/* ノードの数N */
param N integer, >0 ;
/* 分割数T */
param T integer, >0 ;
param link_num ;

set V := {1..N} ;
set K := 1..T ;
set E within {V,V} ;

param c{E} ;
param d{V} ;

/* Decision variable */
var x{E,K} binary ;
var y{V,V,K}, >=0 ;
var b{V,K} binary;
var L{K} ;
var Lmax ;
var DP{V,K} binary;

/*For OR operation*/
var x_or{E,K} binary ;

/* Objective functions */
minimize MIN_LENGTH: sum{(i,j) in E: i != j} (sum{k in K} x_or[i,j,k] - 1)/2 ;

/* Constraints */

s.t. or_constraint1 {(i,j) in E, k in K}: 
     x_or[i,j,k] >= x[i,j,k];
s.t. or_constraint2 {(i,j) in E, k in K}:
     x_or[i,j,k] >= x[j,i,k];
s.t. or_constraint3 {(i,j) in E, k in K}:
     x_or[i,j,k] <= 1;

/* 存在するリンクは必ず通る */
s.t. ST1{(i,j) in E: i != j}:
     1 <= sum{k in K} (x[i,j,k] + x[j,i,k]) ;

/* ノードDから出発するリンクが必ず1つ以上存在する 複数デポの時はルートに含まれていないノードがデポになることを防ぐ*/
s.t. ST2{k in K, i in V}:
     sum{(i,j) in E: j != i} x[i,j,k] >=  DP[i,k] ;

/* ノードに入るリンクの数と出ていく数が同じになる */
s.t. ST3{j in V, k in K}:
     sum{(i,j) in E: i != j} x[i,j,k] - sum{(i,j) in E} x[j,i,k] = 0 ;

/* b_ikとxを関連づける*/
s.t. ST4{i in V, k in K}:
     sum{(i,j) in E} x[i,j,k] + sum{(i,j) in E} x[j,i,k] <= 2 * link_num * b[i,k] ;

s.t. FlowNonDepot_ub{i in V, k in K}:
    (sum{(i,j) in E} y[i,j,k] - sum{(j,i) in E} y[j,i,k]) + b[i,k] <= N * DP[i,k];

s.t. FlowNonDepot_lb{i in V, k in K}:
    (sum{(i,j) in E} y[i,j,k] - sum{(j,i) in E} y[j,i,k]) + b[i,k] >= - N * DP[i,k];

s.t. ST402{(i,j) in E, k in K}    :
     y[i,j,k] <= link_num * x[i,j,k] ;

/* 各ルートの長さを計算する */
s.t. ST5{k in K}:
     L[k] = sum{(i,j) in E} c[i,j]*x[i,j,k] ;

/* Lmaxが各ルートの最大値を取るよう設定する */
s.t. ST6{k in K}:
     Lmax >= sum{(i,j) in E} c[i,j]*x[i,j,k] ;

/* DPで1を立てるところを1つに */
s.t. ST7{k in K}:
     sum{i in V} DP[i,k] = 1 ;

s.t. ST8{i in V, k in K}:
     DP[i,k] <= d[i] ;

solve ;

end ;
