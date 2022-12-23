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
crane_3dmodel.o: crane.h
neural_network.o: nn.h
bat_search.o:crane.h nn.h bat.h
crane_control.o:nn.h crane.h
crane-cs01.o: crane.h nn.h bat.h
