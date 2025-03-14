```math
\check{x}_k
=
\check{\begin{bmatrix}
\theta \\ \dot{\theta}
\end{bmatrix}}_k
=
\begin{bmatrix}
1 & T \\
0 & 1-\frac{b}{J}T
\end{bmatrix} 

\hat{\begin{bmatrix}
\theta \\ \dot{\theta}
\end{bmatrix}}_{k-1}
+
% K here is the motor constant
\begin{bmatrix}
0 \\ \frac{K}{J}T
\end{bmatrix} 

\begin{bmatrix}
v
\end{bmatrix}_{k-1}
```





```math
\check{P}_k
=
\check{\begin{bmatrix}
p_1 & p_2\\
p_3 & p_4
\end{bmatrix}}_{k}
=
\begin{bmatrix}
1 & T \\
0 & 1-\frac{b}{J}T
\end{bmatrix} 

\hat{\begin{bmatrix}
p_1 & p_2\\
p_3 & p_4
\end{bmatrix}}_{k-1}

\begin{bmatrix}
1 & T \\
0 & 1-\frac{b}{J}T
\end{bmatrix}^T
+
\begin{bmatrix}
q_{\theta} & 0\\
0 & q_{\dot{\theta}}
\end{bmatrix}
```

```math
K_k 
=
\check{\begin{bmatrix}
p_1 & p_2\\
p_3 & p_4
\end{bmatrix}}_k
\begin{bmatrix}
1 \\ 0
\end{bmatrix}
(\begin{bmatrix}
 1 & 0
 \end{bmatrix}

 \check{
    \begin{bmatrix}
    p_1 & p_2\\
    p_3 & p_4
    \end{bmatrix}}_k
\begin{bmatrix}
1 \\ 0
\end{bmatrix}
+
\begin{bmatrix}
0.01
\end{bmatrix}   

)^{-1}
```


```math
\hat{x}_k
=
\hat{
    \begin{bmatrix}
    \theta \\ \dot{\theta}
    \end{bmatrix} 
}_{k}
=
\check{
    \begin{bmatrix}
    \theta \\ \dot{\theta}
    \end{bmatrix} 
}_{k}
+
K_k
(
    Z_k
    -
    \begin{bmatrix}
    1 & 0
    \end{bmatrix}
    
    \check{
        \begin{bmatrix}
        \theta \\ \dot{\theta}
        \end{bmatrix}}_k

) 

```


```math
\hat{p}_k
=
(
    \begin{bmatrix}
    1 & 0 \\
    0 & 1
    \end{bmatrix}
    -
    K_k
    \begin{bmatrix}
    1 & 0
    \end{bmatrix}
)

\begin{bmatrix}
p_1 & p_2 \\
p_3 & p_4
\end{bmatrix}_k


```