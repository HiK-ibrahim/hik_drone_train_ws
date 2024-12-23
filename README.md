# hik_drone_train_ws

Bu proje, ROS (Robot Operating System) kullanarak bir drone'u kontrol etmeyi, sensör verilerini işlemeyi ve ray ve zemin arasındaki mesafe tespiti gibi görevler için birden fazla sensörü entegre etmeyi amaçlamaktadır. Projede, LiDAR ve kamera sensörlerinin birleşimiyle füzyon teknolojisi uygulanmış olup, sistemde iki adet kamera ve iki adet 2D LiDAR (ön ve aşağıya bakan) sensörü entegre edilmiştir. Aynı doğrultuda yer alan LiDAR ve kameralar arasında veri füzyonu başarıyla gerçekleştirilmiştir. 

Sistem, drone'u izlemek ve kontrol etmek için bir grafiksel kullanıcı arayüzü (GUI) içermektedir. Arayüzde, birinci görüntüde kamera ve LiDAR füzyonu, ikinci görüntüde ise ray ile zemin arasındaki mesafe ölçümü LiDAR sensörü ile yapılmakta ve görselleştirilmektedir.

## Özellikler
- **Drone Kontrolü**: Drone'un hareketlerini (yukarı, aşağı, ileri, geri, döndürme) kontrol etmek için kullanıcı dostu bir arayüz.
- **Sensör Füzyonu**: Kamera ve LiDAR verilerinin birleştirilmesi ile gelişmiş tespit ve analiz yetenekleri.
- **Ray Tespiti**: Aşağıya bakan kamera ve LiDAR verileri kullanarak ray tespiti yapılmakta, bu özellik otonom tren takibi için kullanılmaktadır.
- **Grafiksel Kullanıcı Arayüzü (GUI)**: Gerçek zamanlı sensör verisi ve füzyon görüntüleri ile drone davranışını izleme imkanı.
- **Modüler Tasarım**: Proje, bağımsız olarak güncellenip test edilebilen çeşitli modüllerle yapılandırılmıştır, bu da geliştirme sürecini kolaylaştırmaktadır.

## Görseller
1. **Füzyon Görüntüsü**: Kamera ve LiDAR füzyonu ve ray ile zemin arasındaki mesafenin görselleştirildiği arayüz görüntüsü.
![image](https://github.com/user-attachments/assets/20bd0cde-990c-44a7-a6fb-df688ba6a44e)

2. **Ray ile Zemin Arasındaki Mesafe Tespiti**: Gazebo test ortamındaki ortam görüntüsü
![image](https://github.com/user-attachments/assets/3b12e8a1-ed3a-4992-b2c2-149cf141d247)

## Gelecek Geliştirmeler
- **Frenet Algoritması**: Otonom drone ve tren takibi için Frener algoritması entegre edilecek.
- **Ray Tespiti ve Takip Algoritmaları**: Ray tespit ve takip algoritmaları geliştirilecek ve optimize edilecek.
- **Kamera ve LiDAR Füzyonu**: Kamera ve LiDAR sensörleri ile ray tespiti yapılacak, bir sensör pasif durumda iken diğer sensör ile devam edilecek şekilde füzyon sağlanacak bu füzyon yapay zeka ile desteklenecek.


