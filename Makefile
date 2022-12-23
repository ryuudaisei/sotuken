# 定義済みマクロの再定義
CC            = gcc
CFLAGS        = -O2 -g -Wno-unused-result
LIBS          = -lm
OBJS          = myfopen.o crane_3dmodel.o neural_network.o bat_search.o crane_control.o crane-cs01.o
PROGRAM       = crane-cs01

.PHONY: all
all:	$(PROGRAM)

# サフィックスルール適用対象の拡張子の定義
.SUFFIXES: .c .o

# プライマリターゲット
$(PROGRAM): $(OBJS)
	$(CC) -o $(PROGRAM) $^ $(LIBS)

# サフィックスルール
.c.o:
	$(CC) $(CFLAGS) -c $<

# ファイル削除用ターゲット
.PHONY: clean
clean:
	$(RM) $(PROGRAM) $(OBJS) *.dat

# ヘッダファイルの依存関係
crane_3dmodel.o: bat.h
neural_network.o: nn.h
genetic_algorithm.o:nn.h ga.h
bat_search.o:bat.h nn.h bat.h
firefly_algorithm.o:bat.h nn.h firefly.h
crane_control.o:nn.h bat.h
crane-ga01.o: bat.h nn.h ga.h
crane-cs01.o: bat.h nn.h bat.h
