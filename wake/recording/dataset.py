from dataclasses import dataclass
@dataclass(frozen=True)
class DatasetSplit:train_session_ids:list[str];validation_session_ids:list[str];test_session_ids:list[str]
def validate_session_split(split:DatasetSplit)->None:
    groups=[set(split.train_session_ids),set(split.validation_session_ids),set(split.test_session_ids)]
    if groups[0]&groups[1] or groups[0]&groups[2] or groups[1]&groups[2]:raise ValueError("recording sessions must not leak across splits")
