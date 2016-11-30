
library(ggplot2)

data <- read.table('dagger_table.txt')
gp <- ggplot(data, aes(x=V1, y=V2))
gp + geom_bar(stat='identity') + labs(title='Success Rates at 100 Training Iterations', x='Learning Type', y='Success %') + theme(axis.text=element_text(size=12), axis.title=element_text(size=14), plot.title=element_text(size=14,face='bold',hjust=0.5))

ggsave('dagger.png')
