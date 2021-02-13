package frc.robot.subClass;

import java.util.function.Function;
import java.util.ArrayList;

/**
 * 制限時間を超えたか超えてないかで処理を切り替えるクラス
 */
public class OriginalTimer {

    // 秒単位
    double start, limtTime;

    // カウントされてるか
    boolean is_Start;
    // 超えたか
    boolean is_Over;

    // 時間を超えた/超えてないときの処理 引数と返り値はどちらも配列
    Function<ArrayList<Object>, ArrayList<Object>> over, notOver;

    /**
     * 制限時間とメソッドの登録.
     * 
     * <p>{@code notOver()}と{@code over()}は{@code ArrayList<Object>}を引数、返り値としてとる関数型インターフェース{@code Function}である。
     * <p>引数が必要なくてもラムダ式には引数を綴っておくこと。
     * 
     * @param limitTime 制限時間 {@code double}型
     * @param notOver 制限時間を超えてないときの関数
     * @param over 制限時間を超えたときの関数
     */
    public OriginalTimer(double limitTime, Function<ArrayList<Object>, ArrayList<Object>> notOver, Function<ArrayList<Object>, ArrayList<Object>> over) {
        this.limtTime = limitTime;
        this.notOver = notOver;
        this.over = over;
        reset();

    }

	public void reset() {
        is_Start = false;
        is_Over = false;
    }

    /**
     * コンストラクタで受け取ったメソッドの実行.
     * 
     * <p>初めて{@code run()}を呼び出したときからの時間に応じて処理を変化させる。
     * 
     * @param notOverArgs 制限時間を超えていないとき({@code notOver()})の引数。{@code CreateArgs()}で作る。
     * @param overArgs 制限時間を超えたとき({@code over()})の引数。{@code CreateArgs()}で作る。
     * @return それぞれのメソッドの返り値
     */
    public ArrayList<Object> run(ArrayList<Object> notOverArgs, ArrayList<Object> overArgs) {

        if (!is_Start) {
            start = System.currentTimeMillis()/1000;
            is_Start = true;
        }

        double nowTime = System.currentTimeMillis()/1000;
        if (nowTime - start < limtTime) {
            return notOver.apply(notOverArgs);
        } else {
            is_Over = true;
            return over.apply(overArgs);
        }

    }

    /**
     * {@code notOver()}も{@code over()}も同じ引数の時.
     * @param args
     * @return それぞれのメソッドの返り値
     */
    public ArrayList<Object> run(ArrayList<Object> args) {
        return run(args, args);
    }

    /**
     * {@code notOver()}も{@code over()}も引数がない時.
     * @return それぞれのメソッドの返り値
     */
    public ArrayList<Object> run() {
        return run(createArgs(), createArgs());
    }

    /**
     * 制限時間を超えたかを返す.
     * @return 制限時間を超えたか
     */
    public boolean is_Over() {
        return is_Over;
    }

    /**
     * 引数を{@code ArrayList}にして返す.
     * <p>異なる型を並べてもよい。
     * 受け取った引数は0-indexedで{@code ArrayList}に並べられる。
     * @param args 引数列
     * @return map {@code ArrayList}に0-indexedに並べられたもの
     */
    public static ArrayList<Object> createArgs(Object... args) {
        ArrayList<Object> map = new ArrayList<Object> ();
        for(int i = 0; i < args.length; i++) {
            map.add(i, args[i]);
        }
        return map;
    }
    /**
     * 引数及び返り値がvoidのときの{@code ArrayList}
     * @return map 空のマップ
     */
    public static ArrayList<Object> createArgs() {
        return new ArrayList<Object> ();
    }

}
