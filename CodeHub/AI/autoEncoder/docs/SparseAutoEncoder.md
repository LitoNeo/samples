# AutoEncoder: 稀疏自动编码器 Sparse_AutoEncoder
---
> 本文为系列文章AutoEncoder第二篇.AutoEncoder对几种主要的自动编码器进行介绍,并使用PyTorch进行实践,相关完整代码将同步到[Github](https://github.com/LitoNeo/pytorch-AutoEncoders)  
本系列主要为记录自身学习历程,并分享给有需要的人.水平所限,错误难免,欢迎批评指正,不吝赐教.  
> 本文及本系列将不定期更新.

AutoEncoder:  
* [自动编码器 AE](https://zhuanlan.zhihu.com/p/83019501)
* [堆叠自动编码器 Stacked AutoEncoder](https://zhuanlan.zhihu.com/p/83331286)
* [稀疏自动编码器 Sparse AutoEncoder]()
* [降噪自动编码器 Denoising AutoEncoder]()
* [变分自动编码器 Variational AutoEncoder]()
* [CAE]()

本文主要包含一下内容:
> 稀疏自动编码器的基本概念,原理,并使用MNIST数据集进行实现

## 1. 基本概念
稀疏自动编码器(SAE)其实就是在普通autoencoder的基础上增加了`稀疏`的约束,使得神经网络在隐藏层神经元较多的情况下依然能够提取样本的特征和结构.

```text
关于稀疏的解释:
    当神经元的的输出接近激活函数上限时(例如对于Sigmoid为1)称该神经元状态为激活,反之当神经元的输出接近激活函数的下限时称该神经元的状态为抑制,那么当某个约束或规则使得神经网络中大部分的神经元的状态为抑制时,称该约束为`稀疏性限制`.
```

## 2. 原理

SAE通常是在损失函数上增加KL散度, 以使得隐藏层神经元输出的平均值接近0(本文未加说明则以Sigmoid函数为例),从而是的大多数神经元处于抑制状态.
> 关于KL散度的直观理解可以参考[这里](https://www.jianshu.com/p/7b7c0777f74d), 以及[这里](https://segmentfault.com/a/1190000012653505)

增加了KL散度后的损失函数为:  
<img src="https://segmentfault.com/img/remote/1460000012653509" />  
公式[引自](https://segmentfault.com/a/1190000012653505)  
$$
J_{\mathrm{sparse}}(W, b)=J(W, b)+\beta \sum_{j=1}^{s_{2}} \mathrm{KL}\left(\rho \| \hat{\rho}_{j}\right)
$$
其中KL散度项为:
$$
\mathrm{KL}\left(\rho \| \hat{\rho}_{j}\right)=\rho \log \frac{\rho}{\hat{\rho}_{j}}+(1-\rho) \log \frac{1-\rho}{1-\hat{\rho}_{j}}
$$

上式中,p表示我们所期望的平均激活值,p_i表示第i个神经元节点的平均激活程度,每一个p_i都会向着p进行靠近.$\beta$为KL散度的权重,通过E来控制KL散度所占的比重.

p_i的求取可以按照下面的式子进行:
$$
\hat{\rho}_{j}=\frac{1}{m} \sum_{i=1}^{m}\left[a_{j}^{(2)}\left(x^{(i)}\right)\right]
$$
其中a_j(x_i)表示第j个神经元在x_i输入下的激活值,m表示输入变量的数量.

**注意:这里所有的稀疏限制都是加在<隐藏层>上的, 因此需要获得隐藏层的输出来计算KL**

由此,我们可以结合PyTorch实现KL散度的计算:
```python
def KL_divergence(p, q):
    """
        calculate the KL-divergence of (p,q)
    """
    p = torch.nn.functional.softmax(p)
    q = torch.nn.functional.softmax(q)
    return torch.sum(p * torch.log(p / q)) + torch.sum((1 - p) * torch.log((1 - p) / (1 - q)))
```

