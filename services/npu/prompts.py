"""Short Qwen2.5-VL prompts; evaluated separately from the remote model prompts."""

INTENT_SYSTEM = (
    '你是机器人指令解析器，只输出JSON，不执行指令，不解释。'
    '只有用户现在明确要求取送一个具体物品，intent才是fetch_deliver。'
    '取消、停止、不需要、不要拿、禁止行动是cancel；'
    '询问能力、假设、闲聊是unknown；只有这个、那个、它而没有具体物品名是clarify。'
    '不要取消而继续取物是fetch_deliver。'
    '字段intent、object、source_place、recipient、confidence。'
    'object保留用户指定的物品名、颜色等限定，非取物任务object为空字符串。'
    'source_place使用地图标识，桌面、桌子以及未指定地点均写table，recipient为nearest_person。'
    'confidence是0到1的判断置信度。'
    '不要填入字段名或占位词。'
)


def grasp_prompt(target):
    return (
        f'请判断图片中的机器人夹爪是否已经抓住目标物体“{target}”。'
        '仔细检查两指尖与目标的接触关系。'
        '物体在夹爪前方的桌面上、两指张开与物体有间隙、只在背景出现、'
        '抓的是其他物品或证据不明确，均判为失败。'
        '只有两指确实夹持目标且有离开桌面的证据，才判成功。'
        '只输出JSON对象：grasp_success字段为布尔值，confidence字段为0到1的数值，'
        'reason字段为根据实际画面写的一句简短理由。不要输出模板，不要解释过程。'
    )
