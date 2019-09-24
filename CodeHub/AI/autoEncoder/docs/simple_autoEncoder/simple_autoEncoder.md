# AutoEncoder: AutoEncoder
---
> 本文为系列文章AutoEncoder第一篇.AutoEncoder对几种主要的自动编码器进行介绍,并使用PyTorch进行实践,相关完整代码将同步到[Github](https://github.com/LitoNeo/pytorch-AutoEncoders)  
本系列主要为记录自身学习历程,并分享给有需要的人.水平所限,错误难免,欢迎批评指正,不吝赐教.  
> 本文及本系列将不定期更新.

AutoEncoder:  
* [自动编码器 AE](https://zhuanlan.zhihu.com/p/83019501)
* [堆叠自动编码器 Stacked AutoEncoder]()
* [稀疏自动编码器 Sparse AutoEncoder]()
* [降噪自动编码器 Denoising AutoEncoder]()
* [变分自动编码器 Variational AutoEncoder]()
* [CAE]()

本文主要包含一下内容:
> 自动编码器的基本概念,注意点,使用MNIST数据集实现最简单的自动编码器

## 1. 自动编码器基本概念
自动编码器(AutoEncoder)是神经网络的一种,一般来讲自动编码器包括两部分:`编码器`和`解码器`,编码器和解码器相互串联合作,实现数据的`降维或特征学习`,现在也广泛用于生成模型中.

在深度学习中，autoencoder可用于在训练阶段开始前，确定权重矩阵的初始值.

放一张最常见的自动编码器的网络结构图:  
![](https://user-images.githubusercontent.com/44689665/65121096-3e915a80-da21-11e9-80e9-411ff8133102.png)  

左侧为encoder,右侧为decoder.可以看出最简单的自动编码器只有三层,输入层+隐藏层x1+输出层.  

自动编码器神经网络是一种无监督机器学习算法，其应用了反向传播，可将目标值设置成与输入值相等。自动编码器的训练目标是将输入复制到输出。在内部，它有一个描述用于表征其输入的代码的隐藏层。

自动编码器可以用一个公式简单的表示:  
f(x) = x

## 2. 需要注意的问题
自动编码器实现了对数据的特征抽象和数据降维,因此需保证输入的数据具有一定的`特征`,即数据之间不是独立同分布的.如果每一个输入都是特征完全无关的,那么就会导致很难学习到特征.  

当每一层都是用线性激活函数来进行激活时,可以发现自编码器可以学习到跟主元分析(PCA)非常接近的效果,从这里也可以看出来自编码器的作用就是用来进行数据降维和特征学习.

## 3. 使用PyTorch+MNIST实现AE
simple AE比较简单,我们直接上代码.
定义网络:
```python
# fully-connected network
class AutoEncoder(nn.Module):
    def __init__(self, in_dim=in_features, hidden_size=hidden_features, out_dim=out_features):
        super(AutoEncoder, self).__init__()
        self.encoder = nn.Sequential(
            nn.Linear(in_features=in_dim, out_features=hidden_size),
            nn.ReLU()
        )
        self.decoder = nn.Sequential(
            nn.Linear(in_features=hidden_size, out_features=out_dim),
            nn.Sigmoid()
        )

    def forward(self, *input):
        out = self.encoder(*input)
        out = self.decoder(out)
        return out
```

定义损失函数等:
```python
Loss = nn.BCELoss()  # 使用二分类交叉熵
Optimizer = optim.Adam(autoEncoder.parameters(), lr=0.001)
```

训练
```python
for epoch in range(num_epochs):
    t_epoch_start = time.time()
    for i, (image_batch, _) in enumerate(data_loader):
        # flatten batch
        image_batch = image_batch.view(image_batch.size(0), -1)
        if torch.cuda.is_available():
            image_batch = image_batch.cuda()
        predict = autoEncoder(image_batch)

        Optimizer.zero_grad()
        loss = Loss(predict, image_batch)
        loss.backward()
        Optimizer.step()

        if (i + 1) % 100 == 0:
            print('Epoch {}/{}, Iter {}/{}, loss: {:.4f}, time: {:.2f}s'.format(
                epoch + 1, num_epochs, (i + 1), len(dataset) // batch_size, loss.data, time.time() - t_epoch_start
            ))
    val_out = autoEncoder(test_images.view(test_images.size(0), -1).cuda())
    val_out = val_out.view(test_images.size(0), 1, 28, 28)
    filename = './data/reconstruct_images_{}.png'.format(epoch + 1)
    torchvision.utils.save_image(val_out, filename)
```

经过20轮迭代后的效果:  
<img src="https://user-images.githubusercontent.com/44689665/65125051-b1053900-da27-11e9-8e08-d0e036564a67.png" />　
<img src="https://user-images.githubusercontent.com/44689665/65125068-b2cefc80-da27-11e9-8d8a-16a7e22228c0.png" />  
可以看出还是有很大的改进空间的.  
详细代码请参考[AE.py](https://github.com/LitoNeo/pytorch-AutoEncoders/blob/master/src/AE.py)

