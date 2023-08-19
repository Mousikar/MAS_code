import matplotlib.pyplot as plt
from wordcloud import WordCloud
from matplotlib.font_manager import FontProperties

# 设置中文显示的字体
font_path = "c:\Windows\Fonts\SIMHEI.TTF"  # 替换为你的中文字体文件路径
fontprop = FontProperties(fname=font_path)

# 输入文本，可以是字符串或者从文件中读取
text = "这是一段文本，用来生成词云图。词云图可以将文本中的单词按照出现频率生成可视化效果。"

# 从文本文件中读取文字
with open('ciyun.txt', 'r', encoding='utf-8') as file:
    text = file.read()

# 合并文本为一行
# text = ' '.join(text.split())

# 创建词云对象
# wordcloud = WordCloud(width=800, height=800, background_color='white', font_path=font_path).generate(text)
wordcloud = WordCloud(width=1200, height=800, background_color='white', font_step=8, max_font_size=80).generate(text)

# 显示词云图
plt.figure(figsize=(12, 8))
plt.imshow(wordcloud, interpolation='bilinear')
plt.axis('off')
plt.show()

# 保存词云图为图片
wordcloud.to_file('wordcloud.png')
