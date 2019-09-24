# AutoEncoder: 堆栈自动编码器 Stacked_AutoEncoder
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
> 堆栈自动编码器的基本概念,原理,并使用MNIST数据集进行实现

## 1. 基本概念
堆栈自动编码器(SAE)也叫深度自动编码器DeepAutoEncoder,从命名上也很容易理解,SAE就是在简单自动编码器的基础上,增加其隐藏层的深度,以获得更好的特征提取能力和训练效果.  

一般来讲, 堆栈自动编码器是关于隐层对称的,如下所示,是一个5层的自动编码器,拥有两个Encoder和两个Decoder:  

<img src="https://user-images.githubusercontent.com/44689665/65301744-467d0600-dbab-11e9-8cb6-148ab3e4982c.png" />

通常Encoder和Decoder的层数是一样的,左右对称.其对称层的参数也可以是具有转置关系的,这种技术称为`权重捆绑`,这样可以使得模型的参数减半,加快训练速度并降低过拟合的风险.  
```注意: 偏置项不进行捆绑```  

## 2. 堆栈自编码器的训练
对于深层模型的训练，通常采用BP算法来更新网络参数。但是需要对网络参数进行很小心的初始化，以免网络陷入局部最小点。当然，现在已经有了很多网络参数初始化的办法，或者其他的深度网络处理技巧，可以很好的避免网络陷入局部最小点，但鉴于`无监督逐层贪婪预训练`在深度网络优化中不可磨灭的影响，我们还是有必要了解这一方法。

对于较深的网络,如果直接进行训练很容易出现梯度消失或梯度爆炸问题,`无监督逐层贪婪预训练`是减缓这一问题的一个简单方法,即通过逐层的进行单独训练,形成一个初始化的参数,在训练第L层时,冻结第1~(L-1)层的参数, 最终实现每一层的单独训练.

代码实现如下:
```python
for epoch_index in range(epoch):
    # 冻结当前层之前的所有层的参数  --第0层没有前置层
    if layer != 0:
        for index in range(layer):
            layers_list[index].lock_grad()
            layers_list[index].is_training_layer = False  # 除了冻结参数,也要设置冻结层的输出返回方式, 见下方解析

    for batch_index, (train_data, _) in enumerate(train_loader):
        # 生成输入数据
        if torch.cuda.is_available():
            train_data = train_data.cuda()  # 注意Tensor放到GPU上的操作方式,和model不同
        out = train_data.view(train_data.size(0), -1)

        # 对前(layer-1)冻结了的层进行前向计算
        if layer != 0:
            for l in range(layer):
                out = layers_list[l](out)

        # 训练第layer层
        pred = layers_list[layer](out)
        optimizer.zero_grad()
        loss = criterion(pred, out)
        sum_loss += loss
        loss.backward()
        optimizer.step()
```

其中每一层隐藏层都需要定义一个单独的输出方式,分别用于进行逐层和训练和全局的训练:  

```python
class AutoEncoderLayer(torch.nn.Module):
    """
    fully-connected linear layers for stacked autoencoders.
    This module can automatically be trained when training each layer is enabled
    Yes, this is much like the simplest auto-encoder
    """

    def __init__(self, input_dim=None, output_dim=None, SelfTraining=False):
        super(AutoEncoderLayer, self).__init__()
        # if input_dim is None or output_dim is None:
        #     raise ValueError
        self.in_features = input_dim
        self.out_features = output_dim
        self.is_training_self = SelfTraining  # 指示是否进行逐层预训练,还是训练整个网络
        self.encoder = torch.nn.Sequential(
            torch.nn.Linear(self.in_features, self.out_features, bias=True),
            torch.nn.Sigmoid()  # 统一使用Sigmoid激活
        )
        self.decoder = torch.nn.Sequential(  # 此处decoder不使用encoder的转置; 使用Sigmoid进行激活.
            torch.nn.Linear(self.out_features, self.in_features, bias=True),
            torch.nn.Sigmoid()
        )

    def forward(self, x):
        out = self.encoder(x)
        if self.is_training_self:  # 当逐层训练时,需要输出self.decoder的计算结果,作为一个输出层; 
            return self.decoder(out)
        else:
            return out

    def lock_grad(self):
        for param in self.parameters():
            param.requires_grad = False

    def acquire_grad(self):
        for param in self.parameters():
            param.requires_grad = True
```

由于每个自编码器都只是优化了一层隐藏层,所以每个隐藏层的参数都只是局部最优的.  
优化完这两个自编码器之后,我们把优化后的网络参数作为神经网络的初始值,之后进行整个网络的训练,直到网络收敛.

完整代码请参考[Github](https://github.com/LitoNeo/pytorch-AutoEncoders/tree/master/src/StackedAutoEncoder)